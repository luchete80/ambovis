import getpass
import os
import base64
import hashlib
import sys
import signal
import subprocess

from Cryptodome.Cipher import AES  # from pycryptodomex v-3.10.4
from Cryptodome.Random import get_random_bytes

HASH_NAME = "SHA256"
IV_LENGTH = 16
ITERATION_COUNT = 65536
KEY_LENGTH = 32

def pad(s): return s + (IV_LENGTH - len(s) % IV_LENGTH) * chr(IV_LENGTH - len(s) % IV_LENGTH)

def unpad(s): return s[0:-ord(s[-1:])]

def get_secret_key(password, salt):
    return hashlib.pbkdf2_hmac(HASH_NAME, password.encode(), salt.encode(), ITERATION_COUNT, KEY_LENGTH)

def encrypt(password, salt, message):
    secret = get_secret_key(password, salt)
    message = pad(message)
    iv = get_random_bytes(IV_LENGTH)
    cipher = AES.new(secret, AES.MODE_CBC, iv)
    cipher_bytes = base64.b64encode(iv + cipher.encrypt(message.encode("utf8")))
    return bytes.decode(cipher_bytes)

def decrypt(password, salt, cipher_text):
    secret = get_secret_key(password, salt)
    decoded = base64.b64decode(cipher_text)
    iv = decoded[:AES.block_size]
    cipher = AES.new(secret, AES.MODE_CBC, iv)
    original_bytes = unpad(cipher.decrypt(decoded[IV_LENGTH:]))
    return bytes.decode(original_bytes)        
 
def fetchPasskey() :
    p = getpass.getpass()
    os.environ["DECRYPT_PWD"] = p
    return p


def decryptData(password, salt):
  with open('dpEncrypted') as dpFile, open('fluxEncrypted') as fluxFile:
    dp_cipher = dpFile.read()
    flux_cipher = fluxFile.read()
    dpDecrypted = decrypt(password, salt, dp_cipher)
    fluxDecrypted = decrypt(password, salt, flux_cipher)
    return dpDecrypted, fluxDecrypted

def setDummyPrivateData():
  with open('ambovis/private_data.h', 'w') as private_data:
    private_data.write('#ifndef AMBOVIS_PRIVATE_DATA_H\n')
    private_data.write('#define AMBOVIS_PRIVATE_DATA_H\n')
    private_data.write('static float dp[] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};\n')
    private_data.write('static byte po_flux[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};\n')
    private_data.write('#endif\n')

def setPrivateData(dpText, fluxText):
  with open('ambovis/private_data.h', 'w') as private_data:
    private_data.write('#ifndef AMBOVIS_PRIVATE_DATA_H\n')
    private_data.write('#define AMBOVIS_PRIVATE_DATA_H\n')
    private_data.write('static float dp[] = {' + dpText + '};\n')
    private_data.write('static byte po_flux[] = {' + fluxText + '};\n')
    private_data.write('#endif\n')

def start():
    try:
        subprocess.call(["arduino-cli", "-v"])
    except FileNotFoundError:
        sys.exit("arduino-cli is not installed")
    f_salt = "respirar_uba"
    pwd=fetchPasskey()
    dp, flux = decryptData(pwd, f_salt)
    setPrivateData(dp, flux)

    print("Compiling...")
    compile_arduino = subprocess.run(['arduino-cli', 'compile', '--fqbn', 'arduino:avr:mega:cpu=atmega2560', 'ambovis', '-v'], 
                         stdout=subprocess.PIPE, 
                         universal_newlines=True)
    compile_arduino
    if (compile_arduino.returncode != 0):
        setDummyPrivateData()
        sys.exit("arduino-cli compilation failed")
    else:
        show_board_list = subprocess.run(['arduino-cli', 'board', 'list'],
                            stdout=subprocess.PIPE, 
                            universal_newlines=True)
        show_board_list
        print(show_board_list.stdout)
        board = input("Copy and paste the port name from the list: ")
        print("Uploading to arduino board " + board)

        arduino_upload = subprocess.run(['arduino-cli', 'upload', '-p', board, '--fqbn', 'arduino:avr:mega:cpu=atmega2560', 'ambovis'],
                            stdout=subprocess.PIPE, 
                            universal_newlines=True)
        if (arduino_upload.returncode == 0):
            print("Upload was successful")
        else:
            print("Error while uploading")

        setDummyPrivateData()        


def handler(signum, frame):
    msg = "Ctrl-c was pressed"
    print(msg, end="", flush=True)
    setDummyPrivateData()
    sys.exit()
 
signal.signal(signal.SIGINT, handler)

start()
