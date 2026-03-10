---
layout: default
title: Control remoto de tira LED usando ESP32, API Flask y Render
nav_order: 7
---

# Control remoto de tira LED usando ESP32, API Flask y Render

En esta práctica se desarrolló un sistema para controlar una tira de LEDs mediante un ESP32 y una interfaz web. El proyecto se implementó en tres etapas. Primero se probó el funcionamiento de forma local utilizando Flask como servidor en la computadora y una página web para enviar los comandos de color y número de LEDs.

Posteriormente se separaron los componentes del sistema, manteniendo el backend en la computadora mientras que el frontend se publicó en GitHub Pages, permitiendo controlar el sistema desde otros dispositivos dentro de la misma red.

Finalmente, el backend se desplegó en la nube mediante Render, mientras que la interfaz web permaneció en GitHub. Con esto se logró enviar información desde la interfaz web hacia el servidor y almacenarla correctamente, demostrando el uso de Flask para lectura y escritura de datos a través de la web.

---

<style>
.acordeon-seccion {
  margin: 1rem 0;
}

.acordeon-seccion details {
  margin-bottom: 12px;
  border: 2px solid #4a4f8c;
  border-radius: 10px;
  background: #4a4f8c;
  overflow: hidden;
}

.acordeon-seccion summary {
  list-style: none;
  cursor: pointer;
  padding: 14px 18px;
  font-size: 1.3rem;
  font-weight: 500;
  color: white;
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.acordeon-seccion summary::-webkit-details-marker {
  display: none;
}

.acordeon-seccion summary::after {
  content: "▾";
  font-size: 1.6rem;
  color: #111;
  border: 1.5px solid #6f72b8;
  width: 32px;
  height: 32px;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  background: #8c8fce;
}

.acordeon-seccion details[open] summary::after {
  content: "▴";
}

.acordeon-contenido {
  background: white;
  padding: 18px 20px;
  border-top: 1px solid #d0d0d0;
}

.acordeon-contenido h3 {
  margin-top: 0.5rem;
}

.acordeon-contenido img {
  max-width: 100%;
  height: auto;
  border-radius: 8px;
}

.boton-descarga {
  display: inline-block;
  padding: 12px 18px;
  background: linear-gradient(135deg, #484D8A, #5E63A8);
  color: white !important;
  border-radius: 12px;
  text-decoration: none;
  font-weight: 600;
  font-size: 0.95rem;
  box-shadow: 0 6px 14px rgba(72,77,138,0.25);
  transition: all 0.2s ease;
}

.boton-descarga:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 18px rgba(72,77,138,0.35);
  text-decoration: none;
}

.acordeon-contenido img{
  display:block;
  margin:15px auto;
  max-width:100%;
  border-radius:8px;
}
</style>

<div class="acordeon-seccion">

<details>
  <summary>Parte 1</summary>
  <div class="acordeon-contenido">

<h3>1. Desarrollo del servidor Flask y página web</h3>

<h4>1.1 Estructura del proyecto</h4>

<p>
Primero se creó la estructura del proyecto dentro de Visual Studio Code,
donde se organizaron los archivos necesarios para el funcionamiento del
servidor Flask y la interfaz web.
</p>

<p>
El proyecto contiene una carpeta llamada <b>static</b>, donde se guardan
los archivos de la interfaz (HTML, CSS y JavaScript). También se incluyen
los archivos del servidor Flask y el archivo donde se almacenará el
estado de los LEDs.
</p>

<p>Los archivos principales del proyecto son:</p>

<ul>
  <li><b>app.py</b> → servidor Flask encargado de manejar las peticiones.</li>
  <li><b>state.json</b> → archivo donde se guarda el estado actual del sistema.</li>
  <li><b>index.html</b> → estructura de la página web.</li>
  <li><b>styles.css</b> → diseño visual de la página.</li>
  <li><b>app.js</b> → lógica de interacción de la página web.</li>
</ul>

<p>
Esta estructura permite separar claramente backend y frontend, facilitando el desarrollo y mantenimiento del sistema.
</p>

<!-- Ejemplo para imagen -->
<img src="{{ '/assets/img/Practica5/1.1.jpeg' | relative_url }}">

<!-- Ejemplo para botón de descarga -->

<a 
  href="{{ '/assets/files/practica5/p5_p1_codsVS.zip' | relative_url }}" 
  download="p5_p1_codsVS.zip"
  class="boton-descarga">
  ⬇ Descargar códigos VS
</a>

<h4>1.2 Configuración del servidor Flask</h4>

<p>
Posteriormente se programó el servidor Flask, el cual es responsable de manejar la comunicación entre la interfaz web y el sistema que controla los LEDs.
</p>

<p>
El servidor cuenta con varias rutas principales:
</p>

<ul>
  <li><code>/</code> → carga la página principal de la interfaz web.</li>
  <li><code>/api/state</code> → permite consultar el estado actual del sistema.</li>
  <li><code>/api/set</code> → permite modificar el color y la cantidad de LEDs encendidos.</li>
  <li><code>/static</code> → permite servir los archivos estáticos de la interfaz.</li>
</ul>

<p>
El servidor también utiliza un archivo llamado state.json para almacenar la información actual del sistema, como el color seleccionado y la cantidad de LEDs activos.
</p>

<p>
Además, el servidor se ejecuta en el puerto 5000, permitiendo que otros dispositivos dentro de la misma red puedan acceder a él.
</p>

<!-- Ejemplo para imagen -->
<img src="{{ '/assets/img/Practica5/1.2.jpeg' | relative_url }}">

<h4>1.3 Comunicación con el servidor</h4>
<p>
Una vez ejecutado el servidor, este comienza a recibir peticiones desde el navegador. En la terminal de Visual Studio Code se pueden observar las solicitudes que realiza la página web al servidor.
</p>

<p>
Estas solicitudes utilizan el método GET para consultar el estado actual del sistema a través de la ruta:
</p>

<p>
/api/state
</p>

<p>
Cada vez que la página se actualiza o consulta el estado, el servidor responde enviando la información almacenada en el archivo state.json.
</p>

<p>
Esto permite mantener sincronizada la interfaz web con el estado real del sistema.
</p>

<h4>1.4 Configuración de red para comunicación con ESP32</h4>
<p>
Para que el microcontrolador ESP32 pudiera comunicarse con el servidor Flask, se configuró la dirección IP de la computadora donde se estaba ejecutando el servidor.
</p>

<p>
Esta dirección IP permite que el ESP32 acceda al servidor a través de la red WiFi y pueda consultar el estado del sistema mediante la ruta:
</p>

<p>
http://IP_DEL_SERVIDOR:5000/api/state
</p>

<p>
En el código del microcontrolador se especifica esta dirección junto con la configuración de la red WiFi a la cual se conecta el dispositivo.
</p>

<img src="{{ '/assets/img/Practica5/1.3.jpeg' | relative_url }}">

<p>
De esta forma, el ESP32 puede obtener la información enviada desde la interfaz web y actualizar el estado de los LEDs.
</p>

<a 
  href="{{ '/assets/files/practica5/cod_colores1.ino' | relative_url }}" 
  download="cod_colores1.ino"
  class="boton-descarga">
  ⬇ Descargar cod_colores1.ino
</a>

<p>
Recuerda que para inicializar el código para que aparezca la IP y la pagina empiece a funcionar debes correr el archivo app.py en terminal así como se muestra en la imagen, das clic derecho y debe salir esta configuración de correr en terminal.
</p>

<img src="{{ '/assets/img/Practica5/1.4.jpeg' | relative_url }}">

<h4>Video de la parte 1</h4>

<video width="600" controls>
  <source src="{{ '/assets/img/Practica5/video1.mp4' | relative_url }}" type="video/mp4">
  Tu navegador no soporta video.
</video>

  </div>
</details>

<details>
  <summary>Parte 2</summary>
  <div class="acordeon-contenido">

<h3>2. Separación del backend local y frontend en GitHub Pages</h3>

<h4>2.1 Cambio de arquitectura del sistema</h4>

<p>
En esta segunda parte ya no se utilizó otra computadora para correr la parte visual del proyecto. En su lugar, se decidió mantener el backend de Flask ejecutándose de forma local en la computadora, mientras que el frontend se publicó en GitHub Pages.
</p>

<p>
Con esta modificación, la interfaz web pudo abrirse desde un repositorio publicado en GitHub, mientras que la lógica de guardado y consulta del estado continuó realizándose desde el servidor Flask local.
</p>

<p>
Esto permitió separar el sistema en dos partes:
</p>

<ul>
  <li><b>Backend local</b> → servidor Flask encargado de guardar y entregar el estado del sistema.</li>
  <li><b>Frontend en GitHub Pages</b> → interfaz web para seleccionar color y cantidad de LEDs.</li>
</ul>

<p>
Esta estrategia evitó ocupar otra computadora únicamente para mostrar el frontend y facilitó el acceso a la interfaz desde otros dispositivos conectados a la misma red.
</p>

<a 
  href="{{ '/assets/img/Practica5/parte2/LEDS_ESP32_PARTE2-main.zip' | relative_url }}" 
  download="LEDS_ESP32_PARTE2-main.zip"
  class="boton-descarga">
  ⬇ Descargar front de GitHub
</a>

<h4>2.2 Ajuste del servidor Flask para permitir acceso externo</h4>

<p>
Para que el frontend publicado en GitHub pudiera comunicarse correctamente con el servidor local, fue necesario modificar el archivo <code>app.py</code>.
</p>

<p>
En esta versión se agregó la librería <code>flask_cors</code> y se habilitó <code>CORS(app)</code>, permitiendo que una página cargada desde otro origen pudiera hacer peticiones al servidor Flask local. También se mantuvo el archivo <code>state.json</code> para guardar el color, la cantidad de LEDs y la fecha de actualización del sistema .
</p>

<p>
Además, el servidor se ejecutó con:
</p>

<ul>
  <li><code>host="0.0.0.0"</code> → para permitir acceso desde otros dispositivos en red.</li>
  <li><code>port=5000</code> → puerto de comunicación del servidor.</li>
  <li><code>ssl_context="adhoc"</code> → para habilitar HTTPS local durante esta etapa del proyecto.</li>
</ul>

<p>
Con esto, el servidor pudo seguir recibiendo y entregando información al frontend, ahora cargado desde GitHub Pages :contentReference[oaicite:2]{index=2}.
</p>

<a 
  href="{{ '/assets/img/Practica5/parte2/p5_p2_codsVS.zip' | relative_url }}" 
  download="p5_p2_codsVS.zip"
  class="boton-descarga">
  ⬇ Descargar códigos VS
</a>

<h4>2.3 Consulta del estado desde la red</h4>

<p>
Una vez configurado el servidor, se comprobó que el estado del sistema podía consultarse desde la red mediante la ruta <code>/api/state</code>. Al abrir esa dirección en el navegador, se mostraba un archivo en formato JSON con la información actual del sistema.
</p>

<p>
Los datos consultados incluían:
</p>

<ul>
  <li><code>color</code> → color seleccionado para los LEDs.</li>
  <li><code>count</code> → cantidad de LEDs encendidos.</li>
  <li><code>updated</code> → fecha y hora de la última actualización.</li>
</ul>

<p>
Esto permitió corroborar que los cambios realizados desde la interfaz web realmente sí estaban llegando al servidor y quedaban almacenados correctamente en el estado del sistema :contentReference[oaicite:3]{index=3}.
</p>

<img src="{{ '/assets/img/Practica5/parte2/2.1.jpeg' | relative_url }}">

<h4>2.4 Comunicación con el ESP32 en esta etapa</h4>

<p>
El microcontrolador continuó consultando el estado del sistema a través del servidor Flask, utilizando la dirección de red correspondiente y el código cargado en Arduino. De esta manera, el ESP32 seguía leyendo el color y la cantidad de LEDs definidos desde la interfaz, pero ahora con el frontend alojado en GitHub Pages.
</p>

<p>
Así, aunque la interfaz ya no corría localmente en la misma computadora, el flujo de comunicación se mantuvo:
</p>

<ul>
  <li>el usuario modificaba el estado desde GitHub Pages,</li>
  <li>Flask lo recibía y lo guardaba en <code>state.json</code>,</li>
  <li>y el ESP32 consultaba ese estado para reflejarlo en los LEDs .</li>
</ul>

<a 
  href="{{ '/assets/img/Practica5/parte2/cod_colores.ino' | relative_url }}" 
  download="cod_colores.ino"
  class="boton-descarga">
  ⬇ Descargar código ESP32
</a>

<h4>Video de la parte 2</h4>

<video width="600" controls>
  <source src="{{ '/assets/img/Practica5/parte2/video2.mp4' | relative_url }}" type="video/mp4">
  Tu navegador no soporta video.
</video>

  </div>
</details>

<details>
  <summary>Parte 3</summary>
  <div class="acordeon-contenido">

### Título de la parte 3

Escribe aquí la descripción o explicación de la tercera parte.

<!-- Ejemplo para imagen -->
<!-- ![Nombre de imagen](assets/img/tu_imagen3.png) -->

<!-- Ejemplo para botón de descarga -->
<!--
<a 
  href="{{ '/assets/files/archivo3.py' | relative_url }}" 
  download="archivo3.py"
  class="boton-descarga">
  ⬇ Descargar archivo3.py
</a>
-->

  </div>
</details>

</div>

---

## Siguiente sección

[Página Web Local](practica4.md)