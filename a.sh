set -e
set -x
virtualenv -p python3 .
source ./bin/activate
pip install --require-hashes -r requirements.txt
