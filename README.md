# CLASE-7-DE-MAYO.
# Control por Rechazo Activo de Perturbaciones (ADRC)

## 📌 Introducción

El **Control por Rechazo Activo de Perturbaciones (ADRC)** es una técnica moderna y robusta que permite controlar sistemas con incertidumbres internas y perturbaciones externas, sin depender de un modelo matemático preciso. ADRC fue introducido formalmente por **Han (2009)** y extendido por **Z. Gao (2003)**.

---

## 🧠 Fundamentos del ADRC

### Características clave (Gao, 2014):
- No requiere un modelo riguroso del sistema.
- Estima la **perturbación total** (modelo + entorno) en tiempo real.
- Induce al sistema a comportarse como una **planta nominal**.

---

## ⚙️ Componentes del ADRC

1. **Generador de Trayectorias (Tracking Differentiator)**  
   Suaviza la referencia y genera sus derivadas.

2. **ESO - Observador de Estado Extendido**  
   Estima:
   - Estados del sistema.
   - Perturbación total.

3. **Controlador de Retroalimentación**  
   Aplica la acción de control compensando perturbaciones.

---

## 🧾 Modelo Matemático

### 📘 Modelo extendido (2º orden):

\[
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f + b_0 u \\
\dot{x}_3 &= \dot{f} = h \\
y &= x_1
\end{aligned}
\]

- \( f \): Perturbación total.
- \( b_0 \): Ganancia crítica nominal.

---

## 🔍 Observador de Estado Extendido (ESO)

### 📘 NADRC (No Lineal):
\[
\begin{aligned}
\dot{z}_1 &= z_2 - \beta_1 \cdot \text{fal}(e, \alpha_1, \delta) \\
\dot{z}_2 &= z_3 + b_0 u - \beta_2 \cdot \text{fal}(e, \alpha_2, \delta) \\
\dot{z}_3 &= -\beta_3 \cdot \text{fal}(e, \alpha_3, \delta) \\
e &= z_1 - y
\end{aligned}
\]

### 📘 LADRC (Lineal):
\[
\begin{aligned}
\dot{z}_1 &= z_2 + L_1 (y - z_1) \\
\dot{z}_2 &= z_3 + b_0 u + L_2 (y - z_1) \\
\dot{z}_3 &= L_3 (y - z_1)
\end{aligned}
\]

---

## 🧮 Ley de Control

### 📘 Control total aplicado:
\[
u = \frac{u_0 - z_3}{b_0}
\]

### 📘 Controlador interno (LADRC):
\[
u_0 = k_1 (r - z_1) - k_2 z_2
\]

---

## 🧠 Diseño de Ganancias

### Para el observador:
\[
L_1 = 3\omega_o, \quad L_2 = 3\omega_o^2, \quad L_3 = \omega_o^3
\]

### Para el controlador:
\[
k_1 = \omega_c^2, \quad k_2 = 2\omega_c
\]

---

## 🧪 Ejemplo: Sistema Masa-Resorte-Amortiguador

### 📘 Ecuación de la planta:
\[
M \ddot{y} + B \dot{y} + Ky = u(t)
\]

### 📘 Espacio de estados:
\[
\begin{aligned}
x_1 &= y \\
x_2 &= \dot{y} \\
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= \frac{1}{M}(u - Bx_2 - Kx_1) = b_0 u + f
\end{aligned}
\]

- Donde \( b_0 = \frac{1}{M} \) y \( f = -\frac{K}{M}x_1 - \frac{B}{M}x_2 \)

---

## 📊 Comparación PID vs ADRC

| Característica     | PID     | ADRC (LADRC) |
|--------------------|---------|--------------|
| Requiere modelo    | Sí      | No           |
| Robustez           | Media   | Alta         |
| Rechazo a perturbaciones | Limitado | Activo en tiempo real |
| Ajuste de parámetros | Empírico | Basado en ωo, ωc |

---

## 🧩 Aplicaciones

- Convertidores DC-DC
- Robots manipuladores
- Sistemas térmicos
- Sistemas multivariables desacoplados (DDC)

---

## 📚 Referencias

- Han, J. (2009). *From PID to ADRC*, IEEE Transactions.
- Gao, Z. (2003). *Active Disturbance Rejection Control: A Paradigm Shift in Control Design*.
- Martínez et al. (2021). *Control por Rechazo Activo de Perturbaciones*, RIAI.

---

> **Nota**: Este documento está basado en apuntes académicos, simulaciones y el artículo “Control por rechazo activo de perturbaciones: guía de diseño y aplicación (RIAI, 2021)”.
