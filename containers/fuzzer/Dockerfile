FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY containers/fuzzer/scripts/fuzzer.py .
RUN chmod +x fuzzer.py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "fuzzer.py"]