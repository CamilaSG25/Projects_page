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
</style>

<div class="acordeon-seccion">

<details>
  <summary>Parte 1</summary>
  <div class="acordeon-contenido">

#### 1Desarrollo del servidor Flask y página web

##### 1.1 Estructura del proyecto

Primero se creó la estructura del proyecto dentro de Visual Studio Code, donde se organizaron los archivos necesarios para el funcionamiento del servidor Flask y la interfaz web.

El proyecto contiene una carpeta llamada static, donde se guardan los archivos de la interfaz (HTML, CSS y JavaScript). También se incluyen los archivos del servidor Flask y el archivo donde se almacenará el estado de los LEDs.

      

<!-- Ejemplo para imagen -->
<!-- ![Nombre de imagen](assets/img/tu_imagen.png) -->

<!-- Ejemplo para botón de descarga -->
<!--
<a 
  href="{{ '/assets/files/archivo1.py' | relative_url }}" 
  download="archivo1.py"
  class="boton-descarga">
  ⬇ Descargar archivo1.py
</a>
-->

  </div>
</details>

<details>
  <summary>Parte 2</summary>
  <div class="acordeon-contenido">

### Título de la parte 2

Escribe aquí la descripción o explicación de la segunda parte.

<!-- Ejemplo para imagen -->
<!-- ![Nombre de imagen](assets/img/tu_imagen2.png) -->

<!-- Ejemplo para botón de descarga -->
<!--
<a 
  href="{{ '/assets/files/archivo2.py' | relative_url }}" 
  download="archivo2.py"
  class="boton-descarga">
  ⬇ Descargar archivo2.py
</a>
-->

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