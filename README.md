```
sudo apt-get install -y libopenblas-dev liblapack-dev libsdl2-dev joystick libatlas-base-dev
```

```
git clone https://github.com/davisking/dlib.git
cd dlib
mkdir build; cd build; cmake .. -DUSE_AVX_INSTRUCTIONS=1; make -j8
sudo make install
cd ..
cd examples
mkdir build; cd build; cmake .. -DUSE_AVX_INSTRUCTIONS=1; make -j8
cd ../..
```

For some reason after install dlib seems to require additional packets - this should probably go away
```
#keys taken from https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB

sudo sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
sudo apt-get update && sudo apt-get install intel-mkl-64bit-2019.1-053
```

install gainput 
```
git clone https://github.com/jkuhlmann/gainput.git
cd gainput 
mkdir build; cd build; cmake ..; make -j8
sudo make install
cd ../..
```


Build tellopp
```
./rebuild.sh
```









