# CLASE-7-DE-MAYO.
# Control por Rechazo Activo de Perturbaciones (ADRC)

## Integrantes 
Carlos Stiven Guezguan         Codigo 96039                           
Juan Jose Martinez             Codigo 105353

## Introducción

El **Control por Rechazo Activo de Perturbaciones (ADRC)** es una técnica moderna y robusta que permite controlar sistemas con incertidumbres internas y perturbaciones externas, sin depender de un modelo matemático preciso. ADRC fue desarrollado por el profesor Jingqing Han en la década de 1990 como una alternativa robusta al control PID tradicional y extendido por **Z. Gao (2003)**.


## Fundamentos del ADRC
ADRC divide el diseño de control en tres bloques principales:

* **Controlador Lineal de Retroalimentación (Linear Feedback Controller):** se utiliza para cerrar el lazo de control con ganancia proporcional.
* **Observador de Estado Extendido (ESO):** su función es estimar tanto el estado como la perturbación total del sistema (incluyendo modelado desconocido).
* **Compensador de Perturbaciones:** emplea la estimación de la perturbación para cancelar activamente su efecto.

Este método se vuelve efectivo incluso en sistemas donde la dinámica completa no está disponible, ya que asume una estructura general y trata el modelado desconocido como una perturbación externa que puede ser estimada y compensada.
### Características clave (Gao, 2014):
- No requiere un modelo riguroso del sistema.
- Estima la **perturbación total** (modelo + entorno) en tiempo real.
- Induce al sistema a comportarse como una **planta nominal**.

## Componentes del ADRC

* **ESO (Extended State Observer):** Estima el estado extendido del sistema, que incluye las perturbaciones y modelado no conocido. Utiliza una estructura de observador de alta ganancia.
* **Controlador de retroalimentación:** Usualmente un PD o un controlador de primer orden, que se encarga de cumplir con los objetivos de seguimiento.
* **Estimación de entrada generalizada (Total Disturbance Estimation):** Se modela como una variable adicional en el observador.

## Modelo Matemático

### Modelo extendido (2º orden):
Para un sistema de segundo orden con perturbaciones, el modelo extendido en espacio de estados se expresa como:
Sistema Dinámico No Lineal:

$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f + b_0 u \\
\dot{x}_3 &= \dot{f} = h \\
y &= x_1
\end{aligned}
$$

donde:

* $x_1 = y$ es la salida del sistema,
* $x_2 = \dot{y}$ es la derivada de la salida,
* $f$ representa la perturbación total (modelo inexacto + perturbaciones externas),
* $b_0$ es la ganancia crítica estimada.

Esta perturbación total se trata como un estado adicional:

$$
\dot{x}_3 = \dot{f} = h \implies
\begin{cases}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= x_3 + b_0 u \\
\dot{x}_3 &= h \\
y &= x_1
\end{cases}
$$

## Observador de Estado Extendido (ESO)

### NADRC (No Lineal):
El ESO tiene como función estimar los estados $x_1$, $x_2$ y $x_3 = f$. El diseño no lineal (NADRC) propone el siguiente conjunto de ecuaciones:

$$
\begin{aligned}
\dot{z}_1 &= z_2 - \beta_1 \gamma_1(e) \\
\dot{z}_2 &= z_3 + b_0 u - \beta_2 \gamma_2(e) \\
\dot{z}_3 &= - \beta_3 \gamma_3(e) \\
e &= z_1 - y
\end{aligned}
$$

donde:

* $z_i$ son las estimaciones de los estados,
* $\gamma_i(e)$ son funciones no lineales tipo "fal",
* $\beta_i$ son las ganancias del observador.

### LADRC (Lineal):
En la variante LADRC, se utiliza un observador lineal tipo Luenberger:

$$
\begin{aligned}
\dot{z}_1 &= z_2 + L_1 (y - z_1) \\
\dot{z}_2 &= z_3 + b_0 u + L_2 (y - z_1) \\
\dot{z}_3 &= L_3 (y - z_1)
\end{aligned}
$$

## Ley de Control

Una vez estimada la perturbación total $\hat{f} \approx z_3$, se compensa en la ley de control:

$$
u = \frac{u_0 - z_3}{b_0}
$$

donde:

$$
u_0 = k_1 (r - z_1) - k_2 z_2
$$

y $r$ es la señal de referencia, $k_1$ y $k_2$ son las ganancias del controlador.

En NADRC, se puede usar una ley de control con funciones no lineales:

$$
u_0 = k_1 \cdot \text{fal}(r - z_1, \alpha_1, \delta) + k_2 \cdot \text{fal}(\dot{r} - z_2, \alpha_2, \delta)
$$

donde:

$$
\text{fal}(e, \alpha, \delta) =
\begin{cases}
\frac{e}{\delta^{1-\alpha}}, & |e| \leq \delta \\
|e|^\alpha \cdot \text{sign}(e), & |e| > \delta
\end{cases}
$$

### Controlador interno (LADRC):
$$
u_0 = k_1 (r - z_1) - k_2 z_2
$$

## Diseño de Ganancias

## Cálculo de Ganancias (LADRC) – Análisis Detallado

**¿Qué se está haciendo en LADRC?**

En el enfoque Lineal del ADRC (LADRC), usamos:

* Un observador lineal de orden 3 (LESO).
* Un controlador de retroalimentación de estados (tipo PD).
* Ambas estructuras se diseñan usando **asignación de polos** basada en dos parámetros clave:
    * $\omega_o$: ancho de banda del observador
    * $\omega_c$: ancho de banda del controlador

**Parte A: Ganancias del Observador (LESO)**

**¿Por qué necesitamos un observador?**

Porque no conocemos la **perturbación total** $f(t)$ ni las **derivadas** del sistema. El LESO la estima en tiempo real junto con los estados del sistema.

**Modelo extendido del sistema (2º orden):**

$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f + b_0 u \\
\dot{f} &= h(t) \quad \text{(desconocido)}
\end{aligned}
$$

Se añade $f$ como un estado adicional: $x_3 = f$

Diseño del observador:

El observador de estado extendido (LESO) se diseña como:

$$
\begin{aligned}
\dot{z}_1 &= z_2 + L_1 (y - z_1) \\
\dot{z}_2 &= z_3 + b_0 u + L_2 (y - z_1) \\
\dot{z}_3 &= L_3 (y - z_1)
\end{aligned}
$$

donde:

* $z_1$ estima $x_1$,
* $z_2$ estima $x_2$,
* $z_3$ estima $x_3 = f(t)$,
* $e = y - z_1$ es el error de observación.

**Parte B: Ganancias del Controlador**

**¿Qué hace el controlador?**

Una vez el observador estima los estados y perturbaciones, diseñamos una ley de control para forzar la salida $y$ a seguir la referencia $r$.

$$
u_0 = k_1 (r - z_1) - k_2 z_2
$$

$$
u = \frac{u_0 - z_3}{b_0}
$$

Este es un controlador de segundo orden (PD).
**Recomendaciones prácticas**

| Parámetro | Valor típico      | Efecto                                    |
|-----------|-------------------|-------------------------------------------|
| $\omega_o$  | 3 a 10 veces $\omega_c$ | Mejor estimación, pero más ruido o esfuerzo |
| $\omega_c$  | 5 – 15 rad/s      | Mayor → respuesta más rápida              |
| $L_i$     | Derivado de $\omega_o$ | Ganancias del observador                |
| $k_i$     | Derivado de $\omega_c$ | Ganancias del controlador               |

### Para el observador:
$$
L_1 = 3\omega_o, \quad L_2 = 3\omega_o^2, \quad L_3 = \omega_o^3
$$

### Para el controlador:
$$
k_1 = \omega_c^2, \quad k_2 = 2\omega_c
$$

## Ejemplo 1

El documento `.docx` proporciona un ejemplo donde se parte de una ecuación de segundo orden tipo resorte-masa-amortiguador:

$$
M\ddot{y} + B\dot{y} + Ky = u(t)
$$

De donde se obtiene la forma en espacio de estados:

$$
\begin{aligned}
x_1 &= y \\
x_2 &= \dot{y} \\
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= \frac{1}{M}(u - Bx_2 - Kx_1)
\end{aligned}
$$

Y el modelo extendido con perturbación total:

$$
\dot{x}_2 = b_0 u + f
$$

donde $f = -\frac{K}{M}x_1 - \frac{B}{M}x_2$ y $b_0 = \frac{1}{M}$.

## Ejemplo 2

## Ecuación de la planta

$$
M\ddot{y} + B\dot{y} + Ky = u(t)
$$

Esta es una ecuación clásica de segundo orden, donde:

* $M$ es la masa del sistema,
* $B$ es el coeficiente de amortiguamiento (fricción),
* $K$ es la constante del resorte,
* $y$ es la posición de la masa,
* $u(t)$ es la fuerza de entrada al sistema.

## Paso a espacio de estados

Definimos las variables de estado:

$$
\begin{aligned}
x_1 &= y \quad &\text{(posición)} \\
x_2 &= \dot{y} \quad &\text{(velocidad)}
\end{aligned}
$$

Entonces:

$$
\dot{x}_1 = x_2
$$

La derivada de la velocidad se obtiene a partir de la ecuación original. Primero despejamos $\ddot{y}$:

$$
M\ddot{y} = u - B\dot{y} - Ky \implies \ddot{y} = \frac{1}{M}(u - B\dot{y} - Ky)
$$

Sustituimos en términos de $x_1$ y $x_2$:

$$
\dot{x}_2 = \frac{1}{M}(u - Bx_2 - Kx_1)
$$

Esto se puede expresar como:

$$
\dot{x}_2 = b_0 u + f
$$

donde:

* $b_0 = \frac{1}{M} \rightarrow$ ganancia crítica del sistema,
* $f = -\frac{B}{M}x_2 - \frac{K}{M}x_1 \rightarrow$ dinámica desconocida del sistema (modelo + perturbación).

## Resumen del modelo en espacio de estados

$$
\begin{aligned}
x_1 &= y \\
x_2 &= \dot{y} \\
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= \frac{1}{M}(u - Bx_2 - Kx_1) = b_0 u + f
\end{aligned}
$$

Este modelo tiene la forma ideal para ser controlado con ADRC, porque:

* $b_0$ puede ser estimado o medido fácilmente (masa conocida),
* $f$ representa la **perturbación total** que el ADRC va a estimar y cancelar.

## Comparación PID vs ADRC

| Característica     | PID     | ADRC (LADRC) |
|--------------------|---------|--------------|
| Requiere modelo    | Sí      | No           |
| Robustez           | Media   | Alta         |
| Rechazo a perturbaciones | Limitado | Activo en tiempo real |
| Ajuste de parámetros | Empírico | Basado en ωo, ωc |

## Aplicaciones del ADRC

**1. Convertidores DC-DC**

Los convertidores como buck o boost presentan dinámica no lineal y son sensibles a cambios en carga o tensión de entrada.

ADRC permite una regulación robusta sin requerir un modelo exacto, compensando perturbaciones en tiempo real.

**2. Robots manipuladores**

Los robots tienen dinámicas complejas, con fricción, acoplamiento y cargas variables.

ADRC mejora el control de cada articulación estimando fuerzas no modeladas y garantizando precisión en el seguimiento.

**3. Sistemas térmicos**

Sistemas como hornos o cámaras térmicas tienen respuesta lenta y están expuestos a perturbaciones externas.

ADRC ofrece una estrategia adaptable, manteniendo estabilidad sin conocer todos los parámetros térmicos.

**4. Sistemas multivariables desacoplados (DDC)**

Cuando múltiples variables están acopladas entre sí (ej. temperatura y humedad), el control se complica.

ADRC trata los efectos cruzados como perturbaciones, permitiendo control canal por canal de forma robusta.

## Conclusiones

* El Control por Rechazo Activo de Perturbaciones (ADRC) es una técnica robusta que permite controlar sistemas con incertidumbres internas, perturbaciones externas y modelos inexactos.
* A diferencia de controladores clásicos como el PID, ADRC no requiere una identificación precisa del modelo y logra una alta calidad de control mediante la estimación y cancelación activa de perturbaciones.
* El uso del Observador de Estado Extendido (ESO) permite estimar dinámicas desconocidas en tiempo real, lo que mejora la capacidad de adaptación del sistema a condiciones cambiantes.
* El enfoque LADRC, por su estructura lineal, resulta práctico para la implementación en sistemas reales, con una sintonización basada en anchos de banda definidos ($\omega_o$ y $\omega_c$).
* Las aplicaciones de ADRC se extienden a sistemas electrónicos, térmicos, robóticos y multivariables, demostrando su versatilidad y efectividad en contextos donde otros métodos fallan.
* En resumen, ADRC representa un avance conceptual y práctico en el diseño de sistemas de control modernos, ideal para situaciones donde el modelado preciso no es viable.

## Referencias

- Han, J. (2009). *From PID to ADRC*, IEEE Transactions.
- Gao, Z. (2003). *Active Disturbance Rejection Control: A Paradigm Shift in Control Design*.
- Martínez et al. (2021). *Control por Rechazo Activo de Perturbaciones*, RIAI.

