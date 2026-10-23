<#
.SYNOPSIS
Copies the Point Cloud Tools macros into a TBC macro folder and checks each copy.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File tools\deploy.ps1 -Target "C:\ProgramData\Trimble\MacroCommands3\PointCloudTools"

The target must be a subfolder of MacroCommands3 that you can write to. Close TBC first, or
restart it afterwards: TBC reads the macros when it starts.
#>
param(
    [Parameter(Mandatory = $true)][string]$Target
)

$source = Join-Path $PSScriptRoot "..\PointCloudTools"
if (Get-Process -Name "TrimbleBusinessCenter" -ErrorAction SilentlyContinue) {
    Write-Warning "TBC is running: restart it after deploying to load the changes."
}
if (-not (Test-Path $Target)) {
    New-Item -ItemType Directory -Path $Target | Out-Null
}
$failed = 0
foreach ($file in Get-ChildItem $source -File | Where-Object { $_.Extension -in ".py", ".xaml" }) {
    $dest = Join-Path $Target $file.Name
    Copy-Item $file.FullName $dest -Force
    $same = (Get-FileHash $file.FullName).Hash -eq (Get-FileHash $dest).Hash
    if (-not $same) { $failed++ }
    "{0,-24} {1}" -f $file.Name, $(if ($same) { "copied" } else { "COPY DIFFERS" })
}
if ($failed) { exit 1 }
