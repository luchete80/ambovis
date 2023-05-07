$global:DP="DEF"
$global:FLUX="DEF"

function fetchPasskey {
  $PASSKEY = Read-Host -AsSecureString 'Enter your pass key: '
  $OPENSSL_PWD = [System.Runtime.InteropServices.Marshal]::PtrToStringAuto([System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($PASSKEY))
  Set-Item -Path Env:OPENSSL_PWD -Value $OPENSSL_PWD
}

function decryptData {
  $global:DP = & openssl enc -in C:\Ardu\ambovis-RESP-137-v2\arduino\EncryptedDp -d -aes256 -pass "env:OPENSSL_PWD" -pbkdf2
  $global:FLUX = & openssl enc -in C:\Ardu\ambovis-RESP-137-v2\arduino\EncryptedFlux -d -aes256 -pass "env:OPENSSL_PWD" -pbkdf2
}

function setDummyPrivateData {
  Set-Location ambovis
  Set-Content private_data.h "
#ifndef AMBOVIS_PRIVATE_DATA_H
#define AMBOVIS_PRIVATE_DATA_H  
static float dp[] = {-2.0, -1.8, -1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2};
static byte po_flux[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
#endif"
  Set-Location ..
}

function closeGracefully() {
  setDummyPrivateData
  Remove-Item -Path Env:OPENSSL_PWD
  exit 1
}

function comienzo {
  if (!(Get-Command 'arduino-cli' -ErrorAction SilentlyContinue)) {
    Write-Host "arduino-cli could not be found"
   exit
  }

  if (-not $env:OPENSSL_PWD) {
    fetchPasskey
  }
  decryptData
  Write-Host 'Decrypting ...'
  Write-Host $global:DP

  Set-Location ambovis/
  Set-Content private_data.h "
    #ifndef AMBOVIS_PRIVATE_DATA_H
    #define AMBOVIS_PRIVATE_DATA_H  
    static float dp[] = {$global:DP};
    static byte po_flux[] = {$global:FLUX};
    #endif"

  Set-Location ..
  
  Write-Host "Compiling..."
  arduino-cli compile  --fqbn arduino:avr:mega:cpu=atmega2560 ambovis -v
  if ($LASTEXITCODE -eq 0) { 
    arduino-cli board list
    $port = Read-Host 'Copy and paste the port name from the list: '

    Write-Host "Uploading to arduino board"
    arduino-cli upload -p $port --fqbn arduino:avr:mega:cpu=atmega2560 ambovis 
    setDummyPrivateData
  } else { 
    closeGracefully
  } 
}

try {
    comienzo
} finally {
    closeGracefully
}