# navegacion-mscthesis
Navegación robusta de vehículos autónomos

Tesis de Maestría

Sonia Dávila

Mapeo:
CoppeliaSim 
- Activar script Ackerman. Correr ambiente y Mappingscript.m, mover con las flechas del teclado. 
- En /Mapa, extracción.m extrae las imágenes de las variables, text1.m genera los txt de asociación
- Correr ORBSLAM2
- plotmap_cloud.m grafica los puntos del txt extraido de ORBSLAM2

Control:
- Trayectorias curvas se generan en /Control/traj_maker.m
- Cambiar puntos finales en rectas de CONTROL_TOUR.m
- Desactivar script Ackerman en AmbienteVirtual
