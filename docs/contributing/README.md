# Cómo contribuir a este proyecto?

La preparación del entorno local puede consultarse [aquí](../manuals/development/README.md) 

### Versionado del Código Fuente

Git es un sistema de control de versiones distribuido, esto es que toda la base de código y el historial se encuentran disponibles en la computadora de cada desarrollador, lo que permite que cada uno pueda gestionar de forma fácil la ramificación y fusión del código. Git como control de versiones permite realizar el seguimiento de cada cambio en el código, además que permite ver el historial de cambios cuando algo en el código sale mal.

#### Estrategia de ramificación _(Branching strategy)_

Una estrategia de ramificación define cómo un equipo utiliza las ramas para lograr un proceso de desarrollo concurrente, a través de un conjunto de reglas y convenciones que establecen:

- ¿Cuándo un desarrollador debe crear una rama _(branch)_?
- ¿De qué otra rama deben derivarse la nueva rama?
- ¿Cuándo debe el desarrollador hacer la fusión _(merge)_?
- ¿Y a qué rama debería hacer la fusión?

Para el caso del proyecto **RespirAR** y en el estadio actual, elegiremos _GitHub Flow_.
#### GitHub Flow

GitHub Flow se basa en un flujo de trabajo basado en ramas que permite a equipos de desarrollo enfocarse principalmente en la entrega continua. A diferencia de Git Flow, no existen los branches de “_releases_”, ya que está pensado para que la implementación en producción ocurra con frecuencia, incluso varias veces al día si es posible.

En el repositorio tendremos dos tipos de branches:

- **_main_** (o **_master_**): la rama de código principal, es el que contiene el código que está listo para producción.
- **features**: las ramas de funcionalidades que permiten el desarrollo en paralelo.

![Github-flow](images/github-flow.png)

### Pasos para crear nuevas ramas

Cada `feature branch` debe salir de `master` realizando los siguientes pasos:

```
git checkout master
git pull origin master
git checkout -b COD-XXX_nombre_descriptivo
```
Los pasos anteriores permiten actualizar localmente la rama `master` con cualquier cambio que otro desarrollador haya hecho y fusionado en el pasado. Entonces al momento de crear una nueva rama para trabajar en una tarea específica, la rama tiene lo último que tenía `master.

La rama `COD-XXX_nombre_descriptivo` debe contener los cambios necesarios para lograr una funcionalidad específica. Ya sea el agregado, borrado o modificación de una funcionalidad existente, o el arreglo de un error.  El nombre es importante ya que permite identificar rápidamente los cambios que contiene la rama. `COD-XXX` es una nomenclatura opcional pero recomendada cuando se trabaja con algún sistema de tickets que asigna a cada tarea pendiente un identificador alfanumérico (por ejemplo Jira). La necesidad de que cada rama tenga el código para algo específico radica en que es más fácil para los demás miembros revisar y entender lo que se quiere hacer.

Se codifica y se hacen las pruebas en la rama `COD-XXX_nombre_descriptivo`. Siempre que sea posible se debe publicar lo trabajado localmente en la rama remota de `COD-XXX_nombre_descriptivo` para que en caso de se desee trabajar desde otra computadora o algún otro miembro del equipo necesite acceder a esa rama específica pueda hacerlo. Esto se logra con:
```
git add <archivos> | git add . { Agrega todos los archivos modificados }
git commit -m "mensaje" { El mensaje debe ser descriptivo de lo que se modificó }
git push origin COD-XXX_nombre_descriptivo
```
Con estos pasos ya tenemos la rama publicada en el servidor. Con esa rama lista, y si es necesario probada en el dispositivo o las pruebas que considere el autor, se puede crear un `pull request` contra la rama `master`. Esto es crear una solicitud de revisión de los cambios para ser fusionados a `master`. Lo debe revisar algún otro miembro del equipo, quien dejará sus comentarios en la herramienta visual de Github o en el mejor de los casos aprobará la solicitud.

### Pasos para crear un Pull request

Ver documentación de [Github](https://docs.github.com/es/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request)

#### Ramas atrasadas o con conflictos
En caso de que un _pull request_ esté abierto por cierto tiempo y otro desarrollador haya fusionado cambios a _master_ con cambios en las mismas líneas de código modificadas por el creador del _pull request_, se mostrará un mensaje de conflicto y no se podrá fusionar hasta que los conflictos estén resueltos. Así mismo, es una buena práctica mantener la rama de trabajo actualizada con los últimos cambios de _master_.

Para hacer estas dos cosas se deben traer los cambios de master a la rama actual:
```
git checkout master
git pull origin master
git checkout COD-XXX_nombre_descriptivo
git merge master or git rebase master
```
Si un mensaje de error es arrojado, hay que resolver los conflictos. Esta [guía](https://docs.github.com/es/github-ae@latest/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line) puede ser útil.

### Protección de la rama master

La importancia de un `pull request` radica en que permite a los desarrolladores enviar sus cambios de código a un repositorio central y solicitar que sus cambios sean incorporados en el proyecto principal. De esta manera, el equipo encargado del mantenimiento del proyecto puede revisar los cambios y asegurarse de que se ajusten a los estándares de calidad y funcionalidad requeridos antes de ser integrados en la rama principal del repositorio.

**Este proyecto está configurado para requerir la aprobación de al menos un miembro de los administradores.**

### Tag y release

Los tags son una forma de identificar versiones específicas de un proyecto y pueden usarse para marcar puntos de referencia importantes, como lanzamientos, versiones estables o hitos específicos del proyecto.

Los administradores del proyecto seran los responsables de crear los tags del código con las contribuciones propias o de terceros. Estas marcas incluyen un numero de version que utilizaran el sistema SemVer (Semantic Versioning). Estas constan de tres números separados por puntos: "Major.Minor.Patch". El número "Major" se incrementa cuando se realizan cambios significativos e incompatibles en la funcionalidad del software. El número "Minor" se incrementa cuando se agregan nuevas funcionalidades compatibles con versiones anteriores, y el número "Patch" se incrementa cuando se corrigen errores y se realizan mejoras menores que no cambian la funcionalidad.

```
git tag <semver> <commit sha>
git tag v1.0.2 HEAD
```

Asi mismo, los administradores decidirán las frecuencias de _release_, es decir, versiones publicadas del software con un _changelog_ asociado. Estas versiones publicadas son basicamente determinados tags que han pasado por diferentes estadíos de pruebas y revisión exitosas.

### Referencias

1. [Estrategias de ramificación](https://openwebinars.net/blog/estrategias-de-branching-gitflow-gitlab-flow-oneflow-github-flow/)
2. [Pull Requests](https://docs.github.com/es/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request)
3. [Conflictos en Pull Requests](https://docs.github.com/es/github-ae@latest/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line)
4. [Releases](https://docs.github.com/es/repositories/releasing-projects-on-github/managing-releases-in-a-repository)
