# Linker wrapper script to filter out --ldproxy-* arguments
$filteredArgs = @()
$skipNext = $false

foreach ($arg in $args) {
    if ($skipNext) {
        $skipNext = $false
        continue
    }
    if ($arg -eq "--ldproxy-linker" -or $arg -eq "--ldproxy-cwd") {
        $skipNext = $true
        continue
    }
    $filteredArgs += $arg
}

& "C:/temp/.embuild/espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc.exe" @filteredArgs
exit $LASTEXITCODE
