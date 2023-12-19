import docker
import signal
import sys
from flask import Flask, render_template, jsonify, request
import time
import threading

template = """
    <!DOCTYPE html>
    <html lang="en">

    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Spes Robotics</title>
    </head>
    <style>
        .control-button
        {
            background-color: #3498db;
            color: white;
            padding: 27px 65px;
            font-size: 30px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            margin: 10px;
            text-decoration: none;
        }
    </style>
    <body>
        <h1>Spes control panel &#129302;</h1>
        <button onclick="start()"
            class="control-button">Start</button>
        <button onclick="stopContainer()" class="control-button">Stop</button>
        <a href="http://192.168.2.146:8080/" target="_blank">
            <button class="control-button">View stream</button>
        </a>
        <hr>
        <h2>Container Status:</h2>
        <div id="status"></div>

        <script>
            function runCuda() {
                fetch('/run_cuda')
                    .then(response => response.json())
                    .then(data => {
                        updateStatus(data);
                    })
                    .catch(error => {
                        updateStatusError(error);
                    });
            }

            function start() {
                fetch('/start')
                    .then(response => response.json())
                    .then(data => {
                        updateStatus(data);
                    })
                    .catch(error => {
                        updateStatusError(error);
                    });
            }

            function stopContainer() {
                fetch('/stop')
                    .then(response => response.json())
                    .then(data => {
                        updateStatus(data);
                    })
                    .catch(error => {
                        updateStatusError(error);
                    });
            }

            function updateStatus(data) {
                const statusElement = document.getElementById('status');
                if (statusElement) {
                    statusElement.innerHTML = `<p><b>Cuda container:</b> ${data.cuda_status}</p><p><b>Green container:</b> ${data.green_status}</p>`;
                } else {
                    console.error("Element with ID 'status' not found.");
                }
            }

            function updateStatusError(error) {
                const statusElement = document.getElementById('status');
                if (statusElement) {
                    statusElement.innerHTML = `<p style="color: red;">Greška prilikom akcije: ${error}</p>`;
                } else {
                    console.error("Element with ID 'status' not found.");
                }
            }

            function checkContainerStatus() {
                fetch("/container_status")
                    .then(response => response.json())
                    .then(data => {
                        updateStatus(data);
                    })
                    .catch(error => {
                        updateStatusError(error);
                    });
            }

            setInterval(checkContainerStatus, 5000);
        </script>
    </body>
    </html>
"""

"""
 docker run --tty --rm --net=host --privileged -v ${PWD}:/home spes-object-tracker-image
  --source 0 --gpus all --model /home/best.pt
"""
"""
  docker run --tty --rm --net=host --privileged -v /dev/dri:/dev/dri:ro -v /dev:/dev:rw -v
  /home/shared/mgk_ws/src/green:/home:rw --name green-container green-deploy-image ros2 run
  spes_move move
"""

CUDA_CONTAINER = "spes-object-tracker"
GREEN_CONTAINER = "green-container"

CUDA_IMAGE = "spes-object-tracker-image"
cuda_params ={
        "tty": True,
        "remove": True,
        "network_mode": "host",
        "privileged": True,
        "volumes": {f"/home/shared/mgk_ws/src/green/cuda": {"bind": "/home"}},
        "name": CUDA_CONTAINER
}
cuda_args = ["--source", "-1", "--gpus", "all", "--model", "/home/best.pt"]

GREEN_IMAGE = "green-deploy-image"
green_params = {
        "tty": True,
        "remove": True,
        "network_mode": "host",
        "privileged": True,
        "volumes": {
            "/dev/dri": {"bind": "/dev/dri", "mode": "ro"},
            "/dev": {"bind": "/dev", "mode": "rw"},
            "/home/shared/mgk_ws/src/green": {"bind": "/home", "mode": "rw"}
        },
        "name": GREEN_CONTAINER
    }
green_args = ["ros2", "run", "spes_move", "move"]

def get_container_logs(client, container_name_or_id):
    try:
        container = client.containers.get(container_name_or_id)
        logs = container.logs().decode("utf-8")
        return logs
    except docker.errors.NotFound as e:
        return f"Container '{container_name_or_id}' not found."
    except docker.errors.APIError as e:
        return f"Failed to get logs for container '{container_name_or_id}': {e}"

def stop_container_by_name(client, container_name):
    try:
        container = client.containers.get(container_name)
        print("Wait to stop container...")
        container.stop()
        print("Wait to remove container...")
        container.remove()
        print(f"Container '{container_name}' stopped and removed.")
    except docker.errors.NotFound:
        print(f"Container '{container_name}' not found.")
    except docker.errors.APIError as e:
        print(f"Failed to stop the container '{container_name}': {e}")

def get_container_status(container_name):
    try:
        client = docker.from_env()
        container = client.containers.get(container_name)
        print(f"Container '{container_name}' status: {container.status}")
        return container.status
    except docker.errors.NotFound:
        return f"stopped"
    except docker.errors.APIError as e:
        print(f"Failed to get status for container '{container_name}': {e}")
        return f"Failed to get status for container '{container_name}': {e}"


def run(image, args, params, container_name):
    client = docker.from_env()
    container = None

    try:
        try:
            stop_container_by_name(client, container_name)
        except Exception as e:
            print(e)
        container = client.containers.run(
            image,
            args,
            detach=True,
            **params
        )
        print(f"{container_name} started successfully.")
        try:
            for log_message in container.logs(stream=True, follow=True):
                print(log_message.decode("utf-8"), end='')
        except Exception as e:
            print(e)

    except docker.errors.ContainerError as e:
        print("Container execution failed with an error:", e)
        print(e.stderr.decode("utf-8"))
    except docker.errors.ImageNotFound as e:
        print("Error: Docker image not found.")
        print(e)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Stopping the container...")
        stop_container_by_name(client, container_name)
        print("Stopped!")
        sys.exit(0)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if container:
            try:
                print("Try to stop processes finaly!")
                exit_code = container.wait()
                print("Container Logs:")
                if exit_code == 0:
                    print("Container executed successfully.")
                else:
                    print(f"Container execution failed with exit code {exit_code}")
            except docker.errors.APIError as e:
                print(f"Failed to get container logs: {e}")
            stop_container_by_name(client, container_name)




def signal_handler(sig, frame):
    print("Server stopped. Stopping container...")
    client = docker.from_env()
    stop_container_by_name(client, CUDA_CONTAINER)
    stop_container_by_name(client, GREEN_CONTAINER)
    print("Stopped container.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

app = Flask(__name__)

@app.route('/')
def index():
   return template

@app.route('/start', methods=['GET'])
def start():
    try:
        thread_object_detection = threading.Thread(target=run, args=(CUDA_IMAGE, cuda_args, cuda_params, CUDA_CONTAINER))
        thread_green_platform = threading.Thread(target=run, args=(GREEN_IMAGE, green_args, green_params, GREEN_CONTAINER))

        thread_object_detection.start()
        thread_green_platform.start()

        thread_object_detection.join()
        thread_green_platform.join()

        return jsonify({'status': 'success', 'message': 'Both Docker containers started!'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/stop', methods=['GET'])
def stop():
    try:
        client = docker.from_env()
        stop_container_by_name(client, CUDA_CONTAINER)
        stop_container_by_name(client, GREEN_CONTAINER)
        return jsonify({'message': 'Stopped and removed containers!'})
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/container_status')
def container_status():
    try:
        client = docker.from_env()
        cuda_status = get_container_status(CUDA_CONTAINER)
        green_status = get_container_status(GREEN_CONTAINER)
        return jsonify({
            "cuda_status": cuda_status,
            "green_status": green_status
        })
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        })


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9000)


