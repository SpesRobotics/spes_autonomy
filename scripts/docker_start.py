import docker
import signal
import sys
from flask import Flask, render_template, jsonify
import time



CONTAINER_NAME = "spes-object-tracker"

"""
 docker run --tty --rm --net=host --privileged -v ${PWD}:/home spes-object-tracker-image
  --source 0 --gpus all --model /home/best.pt
"""

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

def get_container_status(client, container_name_or_id):
    try:
        container = client.containers.get(container_name_or_id)
        if container.status == "running":
            return {"status": "running", "message": "Container is running."}
        else:
            return {"status": "stopped", "message": "Container is stopped."}
    except docker.errors.NotFound:
        return {"status": "not_found", "message": f"Container '{container_name_or_id}' not found."}
    except docker.errors.APIError as e:
        return {"status": "api_error", "message": f"Failed to get container status: {e}"}

def object_detection():
    client = docker.from_env()

    container_params = {
        "tty": True,
        "remove": True,
        "network_mode": "host",
        "privileged": True,
        "volumes": {f"/home/shared/mgk_ws/src/green/cuda": {"bind": "/home"}},
        "name": CONTAINER_NAME
    }

    container = None

    try:
        try:
            stop_container_by_name(client, CONTAINER_NAME)
        except Exception as e:
            print(e)
        container = client.containers.run(
            "spes-object-tracker-image",
            ["--source", "-1", "--gpus", "all", "--model", "/home/best.pt"],
            detach=True,
            **container_params
        )
        print("Container started successfully.")
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
        stop_container_by_name(client, CONTAINER_NAME)
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
            stop_container_by_name(client, CONTAINER_NAME)

def signal_handler(sig, frame):
    print("Server je zaustavljen. Zaustavljanje kontejnera...")
    client = docker.from_env()
    stop_container_by_name(client, CONTAINER_NAME)
    print("Kontejner je zaustavljen.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/run_cuda', methods=['GET'])
def run_cuda():
    try:
        object_detection()
        return jsonify({'status': 'success', 'message': 'Docker started!'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/stop_cuda', methods=['GET'])
def stop_cuda():
    try:
        client = docker.from_env()
        stop_container_by_name(client, CONTAINER_NAME)
        return jsonify({'message': 'Stopped and removed container!'})
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/container_status')
def container_status():
    try:
        client = docker.from_env()
        status_info = get_container_status(client, CONTAINER_NAME)
        return jsonify(status_info)
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9000)


