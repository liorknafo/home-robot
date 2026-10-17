# Stops earlier runs before a Windows launcher starts a new one, so only one of each is ever running.
#   sim       Unreal running HomeRobotSim.uproject: asked to quit over UDP {"quit":true} (port 8767), forced after 10 s.
#             Other Unreal projects are never touched.
#   autonomy  controller\autonomy_service.py
#   vision    vision\vision_service.py
param([Parameter(Mandatory)][ValidateSet('sim', 'autonomy', 'vision')][string]$Kind)

function Find-Runs {
    $all = Get-CimInstance Win32_Process
    switch ($Kind) {
        'sim'      { $all | Where-Object { $_.Name -like 'UnrealEditor*' -and $_.CommandLine -like '*HomeRobotSim.uproject*' } }
        'autonomy' { $all | Where-Object { $_.Name -like 'python*' -and $_.CommandLine -like '*autonomy_service.py*' } }
        'vision'   { $all | Where-Object { $_.Name -like 'python*' -and $_.CommandLine -like '*vision_service.py*' } }
    }
}

$runs = @(Find-Runs)
if ($runs.Count -eq 0) { exit 0 }
Write-Host "Stopping $($runs.Count) previous $Kind process(es): $($runs.ProcessId -join ', ')"

if ($Kind -eq 'sim') {
    $udp = New-Object System.Net.Sockets.UdpClient
    $bytes = [Text.Encoding]::ASCII.GetBytes('{"quit":true}')
    [void]$udp.Send($bytes, $bytes.Length, '127.0.0.1', 8767)
    $udp.Close()
    $deadline = (Get-Date).AddSeconds(10)
    while ((Get-Date) -lt $deadline -and @(Find-Runs).Count -gt 0) { Start-Sleep -Milliseconds 250 }
}

foreach ($p in @(Find-Runs)) { Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue }
$deadline = (Get-Date).AddSeconds(5)
while ((Get-Date) -lt $deadline -and @(Find-Runs).Count -gt 0) { Start-Sleep -Milliseconds 200 }
if (@(Find-Runs).Count -gt 0) { Write-Host "Could not stop every previous $Kind process."; exit 1 }
exit 0
