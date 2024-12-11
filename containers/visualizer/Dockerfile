FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install fastapi uvicorn websockets typing_extensions pyyaml

WORKDIR /app
# Copy the app directory from the container's context
COPY containers/visualizer/app /app/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "main.py"]