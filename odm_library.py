import numpy as np
import matplotlib.pyplot as plt
def square_average(depth_map,position,dim,min_points=10, plot=False):
    #depth_map: matriz de profundidad
    #position: tupla con la posicion del centro del cuadrado
    #dim: tamaño del cuadrado cada lado tiene dim/2 pixeles de distancia
    #min_points: cantidad minima de puntos validos para hacer el promedio
    #plot: si es True muestra el cuadrado de interes en la imagen de profundidad

    row,col=position
    dim=int(dim/2)
    # Busco el area de interes
    start_row = max(0, row - dim)
    end_row = min(depth_map.shape[0], row + dim)
    start_col = max(0, col - dim)
    end_col = min(depth_map.shape[1], col + dim)
    neighbors = depth_map[start_row:end_row,start_col:end_col]
    # Chequeo cuantos vecinos con puntos validos hay, si es mayor a min_points hago el promedio, sino devuelvo Nan 
    if np.count_nonzero(~np.isnan(neighbors)) > min_points:
        average = np.nanmean(neighbors)
    else:
        average = np.nan
    
    # plot
    if plot:
        plt.imshow(depth_map, cmap='jet')
        plt.colorbar()
        # Agrego el cuadrado de interes
        plt.plot([col-dim, col+dim, col+dim, col-dim, col-dim], [row-dim, row-dim, row+dim, row+dim, row-dim], 'r-')
        plt.show()
    
    return average

def circle_average(depth_map,position,rad=10,min_points=10, plot=False):
    #depth_map: matriz de profundidad
    #position: tupla con la posicion del centro del circulo
    #rad: radio del circulo
    #min_points: cantidad minima de puntos validos para hacer el promedio
    #plot: si es True muestra el circulo de interes en la imagen de profundidad
    
    row,col=position
    #Busco area de interés
    neighbors = []
    start_row = max(0, row - rad)
    end_row = min(depth_map.shape[0], row + rad)
    start_col = max(0, col - rad)
    end_col = min(depth_map.shape[1], col + rad)

    for i in range(start_row, end_row):
        for j in range(start_col, end_col):
            if np.sqrt((i-row)**2 + (j-col)**2) <= rad:
                neighbors.append(depth_map[i, j])
    # Chequeo cuantos vecinos con puntos validos hay, si es mayor a min_points hago el promedio, sino devuelvo Nan        
    if np.count_nonzero(~np.isnan(neighbors)) > min_points:
        average = np.nanmean(neighbors)
    else:
        average = np.nan
        
    return average



def scan_average(scan,index,average_size=5, min_points=10):
    #scan: lista de escaneos
    #index: indice del escaneo que quiero promediar
    #average_size: cantidad de escaneos que quiero promediar
    start_index = max(0, index - average_size)
    end_index = min(len(scan), index + average_size)
    scan=scan[start_index:end_index]
    
    if np.count_nonzero(~np.isnan(scan)) > min_points:
        average = np.nanmean(scan)
    else:
        average = np.nan        
    return average