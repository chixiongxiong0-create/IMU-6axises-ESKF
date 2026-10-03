param(
    [string]$VivadoRoot = 'D:/Xilinx/2025.1/Vivado',
    [ValidateSet('spi','acq','invalid','all')][string]$Suite = 'all',
    [string]$BuildDir = '',
    [switch]$InjectFailure
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if (!$BuildDir) { $BuildDir = Join-Path $repoRoot 'build-rtl' }
if (![IO.Path]::IsPathRooted($BuildDir)) { $BuildDir = Join-Path $repoRoot $BuildDir }
function Invoke-Tool([string]$Name, [string[]]$ToolArgs) {
    $binary = Join-Path $VivadoRoot "bin/$Name.bat"
    if (!(Test-Path -LiteralPath $binary)) { throw "Missing Vivado tool: $binary" }
    & $binary @ToolArgs > "$Name.console.log" 2>&1
    if ($LASTEXITCODE -ne 0) {
        Get-Content "$Name.console.log" -Tail 25 | Write-Host
        throw "$Name failed: exit $LASTEXITCODE"
    }
}
$suites = if ($Suite -eq 'all') { @('spi','acq','invalid') } else { @($Suite) }
foreach ($name in $suites) {
    $outDir = Join-Path $BuildDir $name
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
    Push-Location -LiteralPath $outDir
    try {
        $top = switch ($name) {
            'spi' { 'tb_spi_master_3ch' }
            'acq' { 'tb_adxrs453_acq' }
            'invalid' { 'tb_adxrs453_invalid' }
        }
        $rtl = @((Join-Path $repoRoot 'rtl/spi_master_3ch.v'))
        if ($name -ne 'spi') { $rtl += Join-Path $repoRoot 'rtl/adxrs453_acq.v' }
        foreach ($file in $rtl) {
            if (!(Test-Path -LiteralPath $file)) { throw "DUT missing: $file" }
        }
        Invoke-Tool 'xvlog' $rtl
        $tb = @('-sv', (Join-Path $PSScriptRoot "$top.sv"))
        if ($name -eq 'acq') { $tb += Join-Path $PSScriptRoot 'adxrs453_model.sv' }
        Invoke-Tool 'xvlog' $tb
        Invoke-Tool 'xelab' @($top, '-s', "${name}_snapshot", '-debug', 'typical')
        $simArgs = @("${name}_snapshot", '-runall', '-log', 'simulation.log')
        if ($InjectFailure) { $simArgs += @('-testplusarg', 'INJECT_FAILURE') }
        Invoke-Tool 'xsim' $simArgs
        $log = Get-Content 'simulation.log' -Raw
        $failed = $log -match '(?im)(FAIL:|FATAL_ERROR:|Fatal:|Error:|ERROR:)'
        if ($name -eq 'invalid') {
            foreach ($instance in @('c_clk_zero','c_clk_negative','c_spi_zero',
                    'c_spi_negative','c_spi_high','c_half_short','c_gap_short','c_gap_capacity',
                    'c_sample_zero','c_sample_negative','c_capacity')) {
                if ($log -notmatch "(?m)^PARAM_REJECT: [^\r\n]*\.$instance[ :]") { $failed = $true }
            }
        } elseif ($log -notmatch "PASS: $top" -or $log -match 'PARAM_REJECT:') { $failed = $true }
        if ($failed) {
            Get-Content 'simulation.log' -Tail 30 | Write-Host
            throw "Simulation failed or no completion marker: $name"
        }
        Write-Host "PASS: $name ($outDir/simulation.log)"
    } finally { Pop-Location }
}
