FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pandas numpy

WORKDIR /app
COPY containers/recorder/scripts/recorder.py .
RUN chmod +x recorder.py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "recorder.py"]