# app.py
from flask import Flask, request, jsonify
from datetime import datetime, timezone, timedelta

app = Flask(__name__)

registros = []
TZ_MX = timezone(timedelta(hours=-6))  # CDMX fijo -06:00

def ahora_hora_mx():
    # Solo hora (y fecha) para el log; ajusta si quieres solo HH:MM:SS
    return datetime.now(TZ_MX).strftime("%Y-%m-%d %H:%M:%S")

def ahora_iso_mx():
    return datetime.now(TZ_MX).isoformat(timespec="seconds")

def ip_cliente(req):
    xff = req.headers.get("X-Forwarded-For", "")
    if xff:
        return xff.split(",")[0].strip()
    return req.remote_addr

@app.get("/guardar")
def guardar():
    dato = request.args.get("dato", default=None, type=str)

    if dato is None or dato.strip() == "":
        return jsonify({
            "ok": False,
            "error": "Falta el parámetro 'dato'. Ej: /guardar?dato=hola"
        }), 400

    ip = ip_cliente(request)
    ts_iso = ahora_iso_mx()
    ts_log = ahora_hora_mx()

    # Guardamos
    registros.append({"dato": dato, "ip": ip, "ts": ts_iso})

    # Número de mensaje (1,2,3,...)
    n = len(registros)

    # IMPRIMIR EN CONSOLA como quieres:
    print(f"{ip} - {ts_log} - mensaje {n} guardado")

    # Respuesta tipo tu ejemplo (opcional)
    return jsonify({
        "guardado": dato,
        "ip": ip,
        "ok": True,
        "ts": ts_iso
    })

@app.get("/leer")
def leer():
    return jsonify({
        "ok": True,
        "total": len(registros),
        "registros": registros
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
