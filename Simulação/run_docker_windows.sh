docker build -t martha_sim .
docker run --gpus all -it --network=host martha_sim:latest