
rm -rf build
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4

# ./build/bin/GREAT_MSF -x ./sample_data/MSF_20201029/xml/GREAT_MSF_LCPPP_1029.xml