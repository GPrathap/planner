wget https://github.com/cddlib/cddlib/releases/download/0.94j/cddlib-0.94j.tar.gz
tar zxf cddlib-*.tar.gz
cd cddlib-*
./configure
make
sudo make install 
cd ../
sudo rm -rf cddlib-*