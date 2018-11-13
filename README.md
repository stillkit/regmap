# regmap

g++ regmap.cpp -fPIC -shared -o libregmap.so
g++ test.cpp -o test -L. -lregmap
./test > l