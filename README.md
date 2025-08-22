# db-lacam
Fast kinodynamic planner combining LaCam and db-CBS

## Get primitives

The primitives are on the TUB cloud, download a copy, and put them inside db-lacam/

```
wget https://tubcloud.tu-berlin.de/s/wezMej9ieNjwjz6/download
unzip download
rm download
```
## Update the submodule

```
cd dynobench
git submodule update --init --recursive 
```
## Building

Tested on Ubuntu 22.04.

```
mkdir buildRelease
cd buildRelease
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/opt/openrobots/" ..
make -j
```

## Running

```
cd buildRelease
./test-dbpibt -i ../example/swap1_unicycle.yaml -o ../results/pibt.yaml --cfg ../example/algorithms.yaml --t 3000 
```
