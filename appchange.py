from flask import Flask, render_template, redirect, url_for
import subprocess

app = Flask(__name__)

# Diccionario para guardar procesos en ejecucion
processes = {
    "path_and_arucosmakers": None,
    "path_and_arucosmakers": None,
    "path_and_arucosmakers": None,
}

@app.route("/")
def index():
    running = any(p is not None and p.poll() is None for p in processes.values())
    return render_template("hmi.html", running=running)

@app.route("/start/<path_and_arucosmakers>")
def start(modo):
    if modo in processes:
        # Si no esta ya arrancado, lo lanzamos
        if processes[modo] is None or processes[modo].poll() is not None:
            processes[modo] = subprocess.Popen(
                ["python", f"{modo}.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
    return redirect(url_for("hmi.html"))

@app.route("/stop")
def stop():
    # Terminamos TODOS los procesos activos
    for modo, proc in processes.items():
        if proc is not None and proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
            processes[modo] = None
    return redirect(url_for("hmi.html"))

if __name__ == "__main__":
    app.run(debug=True, threaded=True)
