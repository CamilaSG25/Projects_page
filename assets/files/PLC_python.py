from flask import Flask, render_template_string
from pymodbus.client import ModbusTcpClient

# Configuración del PLC
PLC_IP = "192.168.3.154"
PLC_PORT = 502
REGISTER_ADDRESS = 0

app = Flask(__name__)

# Función para escribir al PLC
def write_value(val):
    client = ModbusTcpClient(PLC_IP, port=PLC_PORT)

    try:
        if client.connect():
            result = client.write_register(REGISTER_ADDRESS, val)

            if result.isError():
                return False, f"Error Modbus: {result}"

            if val == 1:
                return True, "🟢 Piloto ENCENDIDO"
            else:
                return True, "🔴 Piloto APAGADO"
        else:
            return False, "⚠️ No se pudo conectar al PLC"

    except Exception as e:
        return False, f"Excepción: {e}"

    finally:
        client.close()

# HTML nuevo (DISEÑO COMPLETO)
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Panel de Control Industrial</title>
    <style>
        body {
            margin: 0;
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #1e1e2f, #2c2c54);
            color: white;
            text-align: center;
        }

        .container {
            margin-top: 80px;
        }

        h1 {
            font-size: 40px;
            margin-bottom: 10px;
        }

        h2 {
            font-weight: normal;
            color: #ccc;
            margin-bottom: 40px;
        }

        .buttons {
            display: flex;
            justify-content: center;
            gap: 40px;
        }

        button {
            width: 220px;
            height: 80px;
            font-size: 20px;
            border: none;
            border-radius: 15px;
            cursor: pointer;
            transition: 0.2s;
        }

        button:hover {
            transform: scale(1.05);
        }

        .on {
            background-color: #00c853;
            color: white;
            box-shadow: 0 0 15px #00c853;
        }

        .off {
            background-color: #d50000;
            color: white;
            box-shadow: 0 0 15px #d50000;
        }

        .msg {
            margin-top: 40px;
            font-size: 22px;
            font-weight: bold;
        }

        .card {
            background-color: rgba(255,255,255,0.05);
            padding: 40px;
            border-radius: 20px;
            display: inline-block;
            box-shadow: 0 0 30px rgba(0,0,0,0.5);
        }
    </style>
</head>
<body>

<div class="container">
    <div class="card">
        <h1>⚙️ Panel de Control</h1>
        <h2>Control remoto del piloto</h2>

        <div class="buttons">
            <form action="/on" method="post">
                <button class="on" type="submit">Prender piloto</button>
            </form>

            <form action="/off" method="post">
                <button class="off" type="submit">Apagar piloto</button>
            </form>
        </div>

        <div class="msg">{{ message }}</div>
    </div>
</div>

</body>
</html>
"""

@app.route("/")
def home():
    return render_template_string(HTML_PAGE, message="Sistema listo")

@app.route("/on", methods=["POST"])
def turn_on():
    ok, msg = write_value(1)
    return render_template_string(HTML_PAGE, message=msg)

@app.route("/off", methods=["POST"])
def turn_off():
    ok, msg = write_value(0)
    return render_template_string(HTML_PAGE, message=msg)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)