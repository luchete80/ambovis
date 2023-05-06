fetchPasskey () {
  read -s -p 'Enter your pass key: ' PASSKEY
  export OPENSSL_PWD=$PASSKEY
}

decryptData () {
  DP=$(openssl enc -in EncryptedDp -d -aes256 -pass "env:OPENSSL_PWD" -pbkdf2)
  FLUX=$(openssl enc -in EncryptedFlux -d -aes256 -pass "env:OPENSSL_PWD" -pbkdf2)
}

setDummyPrivateData() {
  cd ambovis
  rm private_data.h
  echo "
#ifndef AMBOVIS_PRIVATE_DATA_H
#define AMBOVIS_PRIVATE_DATA_H  
static float dp[] = {-2.0, -1.8, -1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2};
static byte po_flux[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
#endif
" > private_data.h
  cd ..
}

start () {
  if ! command -v arduino-cli &> /dev/null
  then
    echo "arduino-cli could not be found"
    exit
  fi
  if [[ -z "${OPENSSL_PWD}" ]]; then
    fetchPasskey
  fi
  decryptData
  echo '
  Decrypting ...'
  echo $DP

  cd ambovis/
  rm private_data.h
  echo "
#ifndef AMBOVIS_PRIVATE_DATA_H
#define AMBOVIS_PRIVATE_DATA_H  
static float dp[] = {$DP};
static byte po_flux[] = {$FLUX};
#endif
" > private_data.h

  cd ..

  echo "Compiling..."
  arduino-cli compile  --fqbn arduino:avr:mega:cpu=atmega2560 ambovis -v
  if [ $? -eq 0 ]; then 
    arduino-cli board list
    read -p 'Copy and paste the port name from the list: ' port 

    echo "Uploading to arduino board"
    arduino-cli upload -p $port --fqbn arduino:avr:mega:cpu=atmega2560 ambovis
    setDummyPrivateData
  else
    setDummyPrivateData
    exit 1
  fi
}

trap ctrl_c INT

function ctrl_c() {
  setDummyPrivateData
  exit
}

start