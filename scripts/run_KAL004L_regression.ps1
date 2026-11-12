# KAL-004L targeted final Kalman regression runner
# Compatible with Windows PowerShell 5.1 and PowerShell 7.
$ErrorActionPreference = "Stop"

$root = (Get-Location).Path
$rtl = Join-Path $root "rtl\kalman_2state_controller_gain18_diag.sv"
$tbDir = Join-Path $root "testbench"
$buildDir = Join-Path $root "build\kal004l"
$logDir = Join-Path $buildDir "logs"

if (!(Test-Path $rtl)) { throw "DUT RTL not found: $rtl" }
if (!(Test-Path $tbDir)) { throw "Testbench directory not found: $tbDir" }

$iverilog = (Get-Command iverilog -ErrorAction Stop).Source
$vvp = (Get-Command vvp -ErrorAction Stop).Source
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

# Explicitly select Kalman regression sources only. Avoid obsolete duplicate variants
# and unrelated LMS/arithmetic-engine tests in this controller sign-off.
$testNames = @(
 "kalman_2state_reference_tb.sv",
 "kalman_2state_fixedpoint_tb.sv",
 "kalman_2state_boundary_tb.sv",
 "kalman_2state_stability_tb.sv",
 "kalman_2state_sequential_tb.sv",
 "kalman_2state_covariance_tb.sv",
 "kalman_2state_random_diag_tb.sv",
 "kalman_2state_random_diag_stage_tb.sv",
 "kalman_2state_gain_range_tb.sv",
 "kalman_2state_small_innovation_tb_fix1.sv",
 "kalman_2state_gain_internal_tb.sv",
 "KAL004J_fixed_point_audit_tb_v2.sv",
 "KAL004K_joseph_form_eval_tb_fix1.sv"
)

$results = @()
foreach ($name in $testNames) {
 $tb = Join-Path $tbDir $name
 if (!(Test-Path $tb)) {
  $results += [pscustomobject]@{Testbench=$name;Top="-";Compile="MISSING";Simulation="NOT_RUN";Detail="Required file not found"}
  Write-Host "MISSING $name" -ForegroundColor Yellow
  continue
 }
 $src = Get-Content -Raw $tb
 $m = [regex]::Match($src, '(?m)^\s*module\s+([A-Za-z_][A-Za-z0-9_$]*)')
 if (!$m.Success) {
  $results += [pscustomobject]@{Testbench=$name;Top="UNKNOWN";Compile="REVIEW";Simulation="NOT_RUN";Detail="Could not infer module name"}
  continue
 }
 $top = $m.Groups[1].Value
 $base = [IO.Path]::GetFileNameWithoutExtension($name)
 $sim = Join-Path $buildDir ($base + ".vvp")
 $outCompile = Join-Path $logDir ($base + ".compile.stdout.log")
 $errCompile = Join-Path $logDir ($base + ".compile.stderr.log")
 $outRun = Join-Path $logDir ($base + ".run.stdout.log")
 $errRun = Join-Path $logDir ($base + ".run.stderr.log")

 Write-Host "COMPILE $name (top=$top)"
 $argsCompile = @("-g2012","-Wall","-s",$top,"-o",$sim,$rtl,$tb)
 $proc = Start-Process -FilePath $iverilog -ArgumentList $argsCompile -Wait -PassThru -NoNewWindow -RedirectStandardOutput $outCompile -RedirectStandardError $errCompile
 if ($proc.ExitCode -ne 0) {
  Write-Host "COMPILE FAIL exit=$($proc.ExitCode)" -ForegroundColor Red
  $results += [pscustomobject]@{Testbench=$name;Top=$top;Compile="FAIL";Simulation="NOT_RUN";Detail="See compiler logs"}
  continue
 }
 Write-Host "RUN $name"
 $runProc = Start-Process -FilePath $vvp -ArgumentList @($sim) -Wait -PassThru -NoNewWindow -RedirectStandardOutput $outRun -RedirectStandardError $errRun
 $runText = ""
 if (Test-Path $outRun) { $runText += Get-Content -Raw $outRun }
 if (Test-Path $errRun) { $runText += Get-Content -Raw $errRun }
 $bad = $runText -match '(?im)\bFAIL\b|\bFATAL\b|ERROR:'
 $good = $runText -match '(?im)ALL .*PASSED|ALL .*PASS|SUMMARY:.*failures\s*=\s*0|checks\s*=\s*\d+.*failures\s*=\s*0'
 if ($runProc.ExitCode -ne 0 -or $bad) {
  $status="FAIL"; Write-Host "SIMULATION FAIL exit=$($runProc.ExitCode)" -ForegroundColor Red
 } elseif ($good) {
  $status="PASS"; Write-Host "PASS" -ForegroundColor Green
 } else {
  $status="REVIEW"; Write-Host "REVIEW: no recognized pass marker" -ForegroundColor Yellow
 }
 $results += [pscustomobject]@{Testbench=$name;Top=$top;Compile="PASS";Simulation=$status;Detail="See $logDir"}
}

$summary = Join-Path $buildDir "KAL-004L-summary.txt"
$passCount = @($results | Where-Object {$_.Compile -eq "PASS" -and $_.Simulation -eq "PASS"}).Count
$failCount = @($results | Where-Object {$_.Compile -eq "FAIL" -or $_.Simulation -eq "FAIL"}).Count
$reviewCount = @($results | Where-Object {$_.Compile -eq "REVIEW" -or $_.Simulation -eq "REVIEW" -or $_.Compile -eq "MISSING"}).Count
$body = @(
 "KAL-004L TARGETED KALMAN REGRESSION",
 "DUT: rtl\kalman_2state_controller_gain18_diag.sv",
 "Required testbenches: $($testNames.Count)",
 "Passed: $passCount",
 "Failed: $failCount",
 "Review/missing: $reviewCount",
 "",
 ($results | Format-Table -AutoSize | Out-String -Width 220)
)
$body | Set-Content -Path $summary
Write-Host ""
Write-Host "KAL-004L SUMMARY: required=$($testNames.Count) passed=$passCount review=$reviewCount failed=$failCount"
Write-Host "Summary: $summary"
Write-Host "Logs: $logDir"
if ($failCount -gt 0) { exit 1 }
if ($reviewCount -gt 0) { exit 2 }
