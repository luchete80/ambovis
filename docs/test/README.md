##Proceso de control de calidad de software

###Estrategia de pruebas elegida

Pruebas unitarias
Herramientas

Test runner:
EpoxyDuino: permite blablab, urL

Librerias:
AUnit: permite blablab, urL
EpoxyEeprom: permite blablab, urL
Más información disponible en la carpeta de documentación [LINK]

###Proceso de control de calidad del software

Un proceso de control de calidad es un conjunto de actividades que se realizan con el fin de garantizar que el software producido cumpla con los estándares y requisitos de calidad establecidos.

El proceso de control de calidad del software puede incluir varias etapas. En primer lugar, se deben establecer los requisitos de calidad para el software. Estos requisitos pueden ser internos o externos, y deben estar claramente definidos para que se puedan medir y evaluar adecuadamente.

Es importante destacar que el proceso de control de calidad del software es un proceso continuo. Es necesario realizar pruebas de calidad regularmente para asegurarse de que el software siga cumpliendo con los requisitos de calidad establecidos. Además, es importante asegurarse de que se están utilizando las mejores prácticas de desarrollo de software y herramientas de prueba de calidad para garantizar que el software producido sea de alta calidad y cumpla con los requisitos del cliente.

El control de calidad del software para sistemas embebidos presenta una serie de desafíos únicos. Estos sistemas se caracterizan por tener restricciones de recursos significativas en términos de memoria, capacidad de procesamiento y consumo de energía.

Uno de los mayores desafíos del control de calidad de software para sistemas embebidos es la necesidad de asegurar la fiabilidad y seguridad del software en situaciones críticas, donde la vida humana puede depender de su correcto funcionamiento. Esto exige un riguroso proceso de verificación y validación del software, que incluye pruebas exhaustivas para asegurar que el software cumpla con los estándares de calidad requeridos.

El costo y la complejidad de las pruebas de calidad del software para sistemas embebidos también son un desafío importante. Dado que el hardware y el software están estrechamente integrados, las pruebas deben incluir tanto pruebas de hardware como pruebas de software, lo que puede ser costoso y requiere recursos especializados.

La propuesta de este proyecto es integrar una serie de prácticas regularmente utilizadas en el desarrollo de otro tipo de sistemas para gozar de los beneficios que brindan.

###Pruebas unitarias

Permiten garantizar que cada unidad de código funcione como se espera y que cualquier cambio posterior no cause problemas en otras partes del sistema. Para lograr esto, se escriben casos de prueba que cubren todos los posibles escenarios de uso y se ejecutan de forma automatizada.

###Pruebas de integración

Gracias a la posibilidad de poder emular la lectura y escritura de los pines se puede generar una interacción entre varios componentes y lograr una integración entre ellos para generar pruebas más robustas. Esto implica un trabajo más exhaustivo en los tests, que podría ser intentar replicar las interacciones que ocurren en el .ino principal y lograr que los tests ocurran por más de un ciclo.

Estos componentes de testeo pueden integrarse al ciclo de desarrollo de la siguiente forma

<img alt="" src="">

###Instalacion y ejecucion local de tests:

Solo en Linux/MacOS debido a limitaciones de las herramientas:
tener instalado git por línea de comandos
en ambovis/arduino ejecutar sh install-dependencies.sh
en una terminal nueva ir a ambovis/arduino/ambovis/tests/unit-tests
ejecutar el comando make runtests

Ejecución de los tests en ambiente de integración continua (Github Actions):
Automáticamente al enviar commit a cualquier branch.


###Aprendizaje sobre Epoxy Duino

Epoxy Duino provee la capacidad de mockear digitalRead y digitalWrite.
Pero no posee la misma funcionalidad para analogRead o analogWrite, lo cual luego de investigar su implementación puede ser algo sencillo de lograr.

La diferencia es que las funciones digital manejan tipo de dato byte (o uint8_t), y para guardar el dato temporal, usa un uint32_t donde cada byte representa un pin (limitando a 32 pines) y donde se puede guardar el valor a devolver..
Si quisiéramos usar la misma estrategia, habría que encontrar la forma de representar el valor a guardar usando dos bytes, ya que analogRead y Write manejan datos de tipo uint16_t (el doble). Esto limitaría la cantidad de pines que se pueden representar o se puede crear una estructura un poco más grande para representar más pines.

Y replicando esta estrategia de "mock" en Epoxy Duino es posible crear un framework más robusto para crear tests unitarios o para probar código sin un dispositivo físico.


