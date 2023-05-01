Frameworks de Desarrollo
-
-

Versionado del Código Fuente

Git es un sistema de control de versiones distribuido, esto es que toda la base de código y el historial se encuentran disponibles en la computadora de cada desarrollador, lo que permite que cada uno pueda gestionar de forma fácil la ramificación y fusión del código. Git como control de versiones permite realizar el seguimiento de cada cambio en el código, además que permite ver el historial de cambios cuando algo en el código sale mal.

Estrategia de Branching
Una “estrategia de branching” es una serie de reglas que aplica un equipo de desarrollo de software cuando necesita escribir un código para incorporar una nueva funcionalidad o hacer una corrección, fusionarlo y enviarlo al repositorio donde se encuentra alojado el resto del código del software en uso, por ejemplo, en un sistema de control de versiones como Git. A su vez, permite que los equipos que trabajan juntos en la misma base de código fuente no afecten su código los unos a los otros.

Una estrategia de branching define cómo un equipo utiliza los branches para lograr un proceso de desarrollo concurrente, a través de un conjunto de reglas y convenciones que establecen:

¿Cuándo un desarrollador debe crear un branch?
¿De qué otro branch deben derivarse el nuevo branch?
¿Cuándo debe el desarrollador hacer merge?
¿Y a qué branch debería hacer el merge?

Trabajar con una sola rama principal es una buena práctica. Y aunque podemos apartarnos de ella, debemos tener claros los motivos por los que necesitamos hacerlo. Nunca debemos olvidar que, sea cual sea nuestro flujo de git, su función principal es apoyar nuestro flujo de release.
Aunque hay muchas opciones disponibles para elegir, para el caso del proyecto RespirAR y en el estadio actual, elegiremos GitHub Flow.

GitHub Flow, fue creado por GitHub y es conocido en la comunidad de desarrolladores como una alternativa simple y ligera a GitFlow. GitHub Flow se basa en un flujo de trabajo basado en branches que permite a equipos de desarrollo enfocarse principalmente en la entrega continua. A diferencia de Git Flow, no existen los branches de “releases”, ya que está pensado para que la implementación en producción ocurra con frecuencia, incluso varias veces al día si es posible.

En esta estrategia de branching, en el repositorio tenemos dos tipos de branches:

main (o master): la rama de código principal, es el que contiene el código que está listo para producción.
features: las ramas de funcionalidades que permiten el desarrollo en paralelo.
Su principal dificultad no está en el propio workflow, sino en el esfuerzo necesario para implementar tests suficientemente confiables y que puedan ser ejecutados con cada pull request.

Esto último es algo que se pretende alcanzar con la etapa 4 del proyecto de refactoring y el framework de testing.

Teniendo en cuenta que el sistema en desarrollo aún no está en producción, sino en una etapa de prueba y solución de errores, no existe la necesidad de versiones de release que puedan ser presentadas o que necesiten ser verificadas. El primer release ocurrirá cuando el sistema pase el periodo de prueba.

Si bien los releases parecen útiles para conservar versiones de código funcionales, la realidad es que aún no se tiene una versión definitiva que sirva de guía o base para agregar nuevas funcionalidades, y tampoco el código se encuentra productivo como para preocuparse por la retrocompatibilidad en caso de cambios. Es decir, si no hay una versión base, si se agregan nuevas funcionalidades pasan a ser parte de la versión base. La clave está en definir los casos de uso de la versión base que deben ser validados para definir el primer release.

Cuando el primer release ocurra, podremos enfocarnos en resolver los problemas que vienen después y tal vez sea momento de moverse a la estrategia de branching GitFlow o alguna otra. Sin embargo, en el estadío actual conviene que el sistema sea lo más simple posible y no mantener tantas ramas sincronizadas. Ya sea porque la adopción resultaría más rápida y porque es mejor no complicar el desarrollo antes de necesitarlo.

![Aquí la descripción de la imagen por si no carga](https://raw.githubusercontent.com/luchete80/ambovis/RESP-141/docs/assets/gitflow.png)

Pasos para Crear nuevos Branch:

Conservar una rama (o branch) master (que a la fecha se encuentra actualizado con develop)
La rama develop deja de existir, y se convierte en una rama como cualquier otra que suele llamarse feature branch.

Cada feature branch debe salir de master realizando los siguientes pasos:

git checkout master
git pull origin master
git checkout -b COD-XXX_nombre_descriptivo

Los pasos anteriores permiten actualizar localmente la rama master con cualquier cambio que otro desarrollador haya hecho y fusionado en el pasado. Entonces al momento de crear una nueva rama para trabajar en un feature específico, la rama tiene lo último que tenía master.

La rama COD-XXX_nombre_descriptivo debe contener los cambios necesarios para lograr una funcionalidad específica. Ya sea el agregado, borrado o modificación de una funcionalidad existente, o el arreglo de un error.  El nombre es importante ya que permite identificar rápidamente los cambios que contiene la rama. COD-XXX es una nomenclatura opcional pero recomendada cuando se trabaja con algún sistema de tickets que asigna a cada tarea pendiente un identificador alfanumérico (por ejemplo Jira).  La necesidad de que cada rama tenga el código para algo específico radica en que es más fácil para los demás miembros revisar y entender lo que se quiere hacer.

Se codifica y se hacen las pruebas en la rama COD-XXX_nombre_descriptivo. Siempre que sea posible se debe publicar lo trabajado localmente en la rama remota de COD-XXX_nombre_descriptivo para que en caso de se desee trabajar desde otra computadora o algún otro miembro del equipo necesite acceder a esa rama específica pueda hacerlo. Esto se logra con:

git add <archivos> | git add . { Agrega todos los archivos modificados }
git commit -m "mensaje" { El mensaje debe ser descriptivo de lo que se modificó }
git push origin COD-XXX_nombre_descriptivo

Con estos pasos ya tenemos la rama publicada en el servidor. Con esa rama lista, y si es necesario probada en el dispositivo o las pruebas que considere el autor, se puede crear un pull request contra la rama master. Esto es crear una solicitud de revisión de los cambios para ser fusionados a master. Lo debe revisar algún otro miembro del equipo, quien dejará sus comentarios en la herramienta visual de Github o en el mejor de los casos aprobará la solicitud.

Pasos para crear un Pull request: https://docs.github.com/es/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request

En caso de que un pull request esté abierto por cierto tiempo y otro desarrollador haya fusionado cambios a master con cambios en las mismas líneas de código modificadas por el creador del pull request, se mostrará un mensaje de conflicto y no se podrá fusionar a master hasta que los conflictos estén resueltos. Así mismo, es una buena práctica mantener la rama de trabajo actualizada con los últimos cambios de master.

Para hacer estas dos cosas se deben traer los cambios de master a la rama actual:
git checkout master
git pull origin master
git checkout COD-XXX_nombre_descriptivo
git merge master

Si un mensaje de error es arrojado, hay que resolver los conflictos. La siguiente guía puede ser útil: https://docs.github.com/es/github-ae@latest/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line

Protección de la rama master

La importancia de un pull request radica en que permite a los desarrolladores enviar sus cambios de código a un repositorio central y solicitar que sus cambios sean incorporados en el proyecto principal. De esta manera, el equipo encargado del mantenimiento del proyecto puede revisar los cambios y asegurarse de que se ajusten a los estándares de calidad y funcionalidad requeridos antes de ser integrados en la rama principal del repositorio.

Además, los pull requests también fomentan la colaboración y el trabajo en equipo, ya que permiten que los desarrolladores revisen y comenten los cambios realizados por sus compañeros. Esto puede ayudar a identificar errores o mejoras potenciales, lo que puede resultar en un código más robusto y de mayor calidad.

En Github, para requerir una aprobación de otro miembro del equipo antes de fusionar un feature branch crearse una regla sobre una rama protegida (master) y marcar las opciones como se muestra abajo:





Referencias:
https://openwebinars.net/blog/estrategias-de-branching-gitflow-gitlab-flow-oneflow-github-flow/

