# MPC

## State space model


$X=
    \begin{bmatrix} x(k) \\ y(k) \\ \theta(k)\end{bmatrix}
     = \begin{bmatrix}
            1 & 0 & 0 \\
            0 & 1 & 0 \\
            0 & 0 & 1
        \end{bmatrix}
        \begin{bmatrix} x(k-1) \\ y(k-1) \\ \theta(k-1) \end{bmatrix}
     + \begin{bmatrix}
            \frac{R}{2}C \cdot dt&  \frac{R}{2}C \cdot dt\\
            \frac{R}{2}S \cdot dt&  \frac{R}{2}S \cdot dt\\
            \frac{R}{L} \cdot dt&  -\frac{R}{L} \cdot dt\end{bmatrix}
        \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}
        $
        
$C = cos(\theta(k-1))$
$S = sin(\theta(k-1))$