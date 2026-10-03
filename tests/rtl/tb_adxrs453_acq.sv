`timescale 1ns/1ps
module tb_adxrs453_acq;
    reg clk=0, rst_n=0, power_reset_n=0;
    always #5 clk=~clk;
    wire sclk,mosi; wire [2:0] cs_n;
    wire [2:0] miso;
    reg stuck_en=0, stuck_value=0;
    wire [2:0] connected_miso={miso[2:1],stuck_en ? stuck_value : miso[0]};
    wire signed [15:0] gyro0,gyro1,gyro2;
    wire [31:0] raw0,raw1,raw2;
    wire [63:0] timestamp_cycles;
    wire [2:0] frame_seq,gyro_valid,startup_ok;
    wire [4:0] err0,err1,err2;
    wire frame_valid,startup_done,overrun;
    adxrs453_acq dut(.miso(connected_miso),.*);
    reg [15:0] rate[0:2];
    reg [31:0] flips[0:2];
    reg [2:0] sq[0:2]; reg sq_en[0:2];
    reg [1:0] st[0:2]; reg st_en[0:2];
    reg [6:0] faults[0:2];
    genvar g;
    generate for(g=0;g<3;g=g+1) begin: sensors
        adxrs453_model model(.power_reset_n(power_reset_n),.cs_n(cs_n[g]),
            .sclk(sclk),.mosi(mosi),.miso(miso[g]),.rate(rate[g]),
            .flip_mask(flips[g]),.override_sq(sq_en[g]),.sq_value(sq[g]),
            .override_st(st_en[g]),.st_value(st[g]),.fault_value(faults[g]));
    end endgenerate
    task check(input bit ok, input string msg);
        if (!ok) begin $display("FAIL: acquisition %s at %0t",msg,$time); $fatal(1); end
    endtask
    integer startup_exchanges=0, run_exchanges=0, frames=0, resets=0;
    reg [63:0] cycles=0, expected_stamp=0, origin_stamp=0, previous_stamp=0;
    reg [15:0] expected_rate[0:2];
    reg [2:0] expected_seq=0;
    reg [31:0] command=0;
    realtime released_at=0, startup_low, previous_startup_high=0;
    reg monitor_period=1;
    reg [2:0] wanted_startup=7;
    integer i;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) cycles<=0; else cycles<=cycles+1;
    always @(negedge rst_n) begin
        startup_exchanges=0; run_exchanges=0; frames=0;
        expected_seq=0; previous_stamp=0; origin_stamp=0;
    end
    always @(posedge rst_n) released_at=$realtime;
    always @(negedge cs_n[0]) if(rst_n) begin
        check(cs_n===0,"three CS aligned"); command=0;
        if (!startup_done) begin
            startup_low=$realtime;
            if(startup_exchanges==0) check($realtime-released_at>=100000000,"100 ms startup");
            if(startup_exchanges==1 || startup_exchanges==2)
                check($realtime-previous_startup_high>=50000000,"50 ms CHK wait");
        end else begin
            run_exchanges=run_exchanges+1;
            if(run_exchanges%2==1) begin
                expected_stamp=cycles;
                for(integer k=0;k<3;k=k+1) expected_rate[k]=rate[k];
                // Change after A has latched, before B begins its exchange.
                #1; for(integer k=0;k<3;k=k+1) rate[k]=rate[k]+16'd17;
            end else begin
                check(rate[0]!==expected_rate[0],"A and B latch data must differ in model");
            end
        end
    end
    always @(posedge sclk) if(rst_n && cs_n==0) command={command[30:0],mosi};
    always @(posedge cs_n[0]) if(rst_n) begin
        if(!startup_done) begin
            check(command==((startup_exchanges==0)?32'h20000003:32'h20000000),"startup command");
            startup_exchanges=startup_exchanges+1; previous_startup_high=$realtime;
        end else begin
            check({command[28],command[31:30]}===expected_seq,"command SQ encoding");
        end
    end
    always @(posedge startup_done) if(rst_n) begin
        check(startup_exchanges==5,"five startup exchanges");
        check(startup_ok===wanted_startup,"per-axis startup result");
    end
    reg [63:0] published_stamp;
    reg [95:0] published_raw;
    reg [47:0] published_gyro;
    reg [20:0] published_flags;
    reg prev_valid=0;
    always @(negedge clk) if(rst_n) begin
        if(frame_valid) begin
            check(startup_done,"no startup frames");
            check(!prev_valid,"single-cycle frame pulse");
            check(run_exchanges%2==0,"publish after B");
            check(timestamp_cycles===expected_stamp,"A latch timestamp");
            check(frame_seq===expected_seq,"frame SQ");
            if(!stuck_en && flips[0]==0 && !sq_en[0] && !st_en[0] && faults[0]==0)
                check(gyro0===expected_rate[0],"A data axis 0");
            check(gyro1===expected_rate[1] && gyro2===expected_rate[2],"A data axes 1/2");
            if(monitor_period && frames>0) begin
                check(timestamp_cycles-previous_stamp==206185 || timestamp_cycles-previous_stamp==206186,"485 Hz interval");
                check((timestamp_cycles-origin_stamp)*485 >= frames*100000000-484 &&
                      (timestamp_cycles-origin_stamp)*485 <= frames*100000000+484,"cumulative phase error <1 cycle");
            end
            if(frames==0) origin_stamp=timestamp_cycles;
            previous_stamp=timestamp_cycles; frames=frames+1; expected_seq=expected_seq+1;
        end else if(prev_valid) begin
            published_stamp=timestamp_cycles; published_raw={raw2,raw1,raw0};
            published_gyro={gyro2,gyro1,gyro0}; published_flags={frame_seq,gyro_valid,err2,err1,err0};
        end else if(startup_done && frames>0) begin
            check(timestamp_cycles===published_stamp && {raw2,raw1,raw0}===published_raw &&
                  {gyro2,gyro1,gyro0}===published_gyro &&
                  {frame_seq,gyro_valid,err2,err1,err0}===published_flags,"atomic outputs retained");
        end
        prev_valid=frame_valid;
    end else prev_valid=0;
    task next_frame;
        @(posedge frame_valid); #2;
    endtask
    task axis0_error(input [4:0] expected);
        next_frame();
        check(err0===expected && gyro_valid===3'b110,"axis0 error mask / isolation");
        check(err1==0 && err2==0,"other axes healthy");
        @(negedge frame_valid);
    endtask
    initial begin
        for(i=0;i<3;i=i+1) begin
            rate[i]=i*80; flips[i]=0; sq[i]=0; sq_en[i]=0;
            st[i]=0; st_en[i]=0; faults[i]=0;
        end
        #1 power_reset_n=1;
        repeat(30) @(negedge clk); rst_n=1;
        wait(startup_done); #2;
        $display("CHECK: full 100/50/50 ms startup passed");
        next_frame(); check(gyro_valid==7 && err0==0 && err1==0 && err2==0,"normal frame");
        @(negedge frame_valid);
        rate[0]=16'h0050; rate[1]=16'hffb0; rate[2]=0;
        next_frame(); check(gyro0==80 && gyro1==-80 && gyro2==0,"signed values");
        @(negedge frame_valid);
        rate[0]=16'h7fff; rate[1]=16'h8000; rate[2]=16'hffff;
        next_frame(); check(gyro0==32767 && gyro1==-32768 && gyro2==-1,"signed boundaries");
        repeat(16) next_frame();
        @(negedge frame_valid);
        // Isolate P0: flip one high-half bit and P1, keeping overall odd parity.
        flips[0]=32'h10000001; axis0_error(5'b00001); flips[0]=0;
        flips[0]=1; axis0_error(5'b00010); flips[0]=0;
        sq_en[0]=1; sq[0]=expected_seq^3'b001; axis0_error(5'b00100); sq_en[0]=0;
        st_en[0]=1;
        st[0]=0; axis0_error(5'b01000);
        st[0]=2; axis0_error(5'b01000);
        st[0]=3; axis0_error(5'b01000); st_en[0]=0;
        for(i=0;i<7;i=i+1) begin faults[0]=7'b1<<i; axis0_error(5'b10000); end
        faults[0]=0;
        stuck_en=1; stuck_value=0; next_frame(); check(!gyro_valid[0],"MISO stuck low");
        @(negedge frame_valid); stuck_value=1;
        next_frame(); check(!gyro_valid[0],"MISO stuck high");
        @(negedge frame_valid); stuck_en=0;
        next_frame(); check(gyro_valid==7,"fault recovery");
        // Retain sensor power; only FPGA resets. Make axis 2 fail startup parity.
        @(negedge frame_valid); @(negedge clk); rst_n=0; wanted_startup=3;
        flips[2]=1;
        repeat(30) @(negedge clk); rst_n=1;
        wait(startup_done); #2; flips[2]=0;
        next_frame(); // First A overwrites previously corrupted pending response.
        check(startup_ok==3 && gyro_valid==3,"startup failure remains gated");
        @(negedge frame_valid);
        // Overrun: stall completion of an existing A transfer for one tick.
        monitor_period=0;
        wait(cs_n==0); force dut.spi_done=1'b0;
        repeat(210000) @(negedge clk);
        check(overrun,"busy tick overrun");
        // Sequence advances over a dropped tick; existing frame retains its own SQ.
        force dut.spi_done=1'b1; @(negedge clk); release dut.spi_done;
        next_frame(); @(negedge frame_valid);
        expected_seq=expected_seq+1;
        next_frame(); check(overrun,"overrun sticky");
        // Abort active exchange; full startup is repeated without sensor power reset.
        wait(cs_n==0); repeat(5) @(posedge sclk); @(negedge clk); rst_n=0;
        #1; check(cs_n==7 && sclk==0 && !frame_valid && !startup_done && !overrun,"reset outputs");
        wanted_startup=7; monitor_period=1;
        repeat(30) @(negedge clk); rst_n=1;
        wait(startup_done); next_frame(); check(gyro_valid==7,"reset recovery");
        $display("PASS: tb_adxrs453_acq"); $finish;
    end
    initial begin #1000000000; $display("FAIL: acquisition watchdog"); $finish; end
endmodule
