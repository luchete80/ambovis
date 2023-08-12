# Proyecto RespirAR UBA

## Ventilador Mecánico Mínimo (VMM)
El VMM es un proyecto nacido en la Facultad de Ingeniería de la Universidad de Buenos Aires y tiene como objetivo crear un prototipo de respirador de bajo costo. [Leer más](https://www.fi.uba.ar/noticias/desarrollo-de-insumos-y-equipamiento-medico-3984)

En este repositorio encontrará el codigo escrito en C++ junto con la documentación de:
- [Manuales](docs/manuals/README.md)
  - [Preparación del entorno y ejecución del código](docs/manuals/development-setup.md)
  - [Uso del menú del dipositivo](docs/manuals/menu/README.md)
- [Recursos de investigación](docs/resources/README.md)
  - [Esquemáticos](docs/resources/schematics)
  - [Recursos de lectura](docs/resources/help%20sources)
  - [Simulador arduino tft](docs/resources/tft_simulator.md)
- [Calidad de software](docs/test/README.md)
  - [Pruebas manuales](docs/test)
  - [Pruebas unitarias](docs/test/unit_tests.md)
- [Contribución](docs/contributing/README.md)

## Funcionalidades

Las funcionalidades cubiertas por el sistema hasta el momento son:
- Funcionamiento del sistema con alimentación alterna o por batería
- Modo reposo del sistema
- Configuración de parámetros: BPM, IE, Volumen
- Configuración de valores de alarmas: PIP, PEEP, VM
- Visualización de parametros: PIP, PEEP, VM
- Visualización de gráficas de presión y flujo
- Visualización de carga de batería
- Visualizacion de alarmas: en pantalla, sonido y por indicador luminoso
- Guardado en memoria de ciertas configuraciones

## Contacto

Por consultas, enviar un email a [respirar@fi.uba.ar](mailto:respirar@fi.uba.ar?subject=[RespirAR])
