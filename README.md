# sdn-vanet
SDN over VANET

NEST on NS-3

GET ns-3 by

    ~$ wget https://www.nsnam.org/release/ns-allinone-3.25.tar.bz2
    ~$ tar xjf ns-allinone-3.25.tar.bz2


place this repository by

    ~$ mv ns-allinone-3.25/ns-3.25/ ~/
    ~$ cd ns-3.25/
    ~/ns-3.25$ git init
    ~/ns-3.25$ git pull https://github.com/CHLLHC/sdn-vanet.git


configure by

    ~/ns-3.25$ CXXFLAGS="-Wall -g -std=c++11" ./waf -d debug --enable-examples configure
    
build by

    ~/ns-3.25$ ./waf
    
OR

    ~/ns-3.25$ ./waf -j4
      (For 4 threads computer)
    ~/ns-3.25$ ./waf -j8
      (For 8 threads computer)

run by

    ~/ns-3.25$ ./waf --run "SDN"
