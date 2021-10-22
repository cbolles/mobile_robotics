import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import math

MAPSIZE = 1000
POINTS_TO_CHECK = 50

class Visualizer(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)

        # Initialize visualization
        self.master.title('Distribution Visualization')
        self.master.minsize(width=MAPSIZE,height=MAPSIZE)
        
        self.themap = Image.new("L", (MAPSIZE,MAPSIZE), 128)
        self.mapimage = ImageTk.PhotoImage(self.themap)

        # Memory access to image pixels
        self.mappix = self.themap.load()

        for x in range(0, MAPSIZE):
            for y in range(0, MAPSIZE):
                self.mappix[x, y] = 256

        self.canvas = tk.Canvas(self,width=MAPSIZE, height=MAPSIZE)

        self.map_on_canvas = self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)
        self.canvas.pack()
        self.pack()

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)       
        self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)

    def draw_gaussian(self, mean_x, mean_y, std_dev_x, std_dev_y, rotation):
        # Matrix of standard deviation assuming no rotation
        c_matrix = np.matrix([
                [pow(std_dev_x, 2), 0],
                [0, pow(std_dev_y, 2)]])
        # Rotation matrix to transform C
        rotation_matrix = np.matrix([
                [math.cos(rotation), -1 * math.sin(rotation)],
                [math.sin(rotation), math.cos(rotation)]])
        # Covariance relationship
        sigma = np.matmul(np.matmul(rotation_matrix, c_matrix), np.transpose(rotation_matrix))
        # Matrix representing the position of the center mean
        pos_mean_matrix = np.matrix([
                [mean_x],
                [mean_y]])
        
        # Iterate over the points to calculate the probability for about the mean
        start_x = int(mean_x - POINTS_TO_CHECK / 2)
        start_y = int(mean_y - POINTS_TO_CHECK / 2)
        end_x = int(mean_x + POINTS_TO_CHECK / 2)
        end_y = int(mean_y + POINTS_TO_CHECK / 2)

        print(start_x, start_y)
        print(end_x, end_y)
        for x in range(start_x, end_x + 1):
            for y in range(start_y, end_y + 1):
                pos_matrix = np.matrix([
                        [x],
                        [y]])
                
                probability_base = 1 / 2 * math.pi * np.sqrt(np.linalg.det(sigma))
                probability_exp = (-0.5 * np.transpose(pos_matrix - pos_mean_matrix)) * np.linalg.inv(sigma) * (pos_matrix - pos_mean_matrix)
                probability = np.power(probability_base, probability_exp)[0, 0]

                # Transform (0, 0) to center of map
                vis_x = x + int(MAPSIZE / 2)
                vis_y = y + int(MAPSIZE / 2)

                # Bounds checking
                if vis_x < 0 or vis_y < 0 or vis_x >= MAPSIZE or vis_y >= MAPSIZE:
                    continue
                
                # print(x, y, probability)
                # Convert to a color representation
                self.mappix[vis_x, vis_y] = int(probability * 256)


def main():
    root = tk.Tk()
    visual = Visualizer(master=root,height=MAPSIZE,width=MAPSIZE)

    visual.draw_gaussian(0, 0, 10, 20, math.radians(45))

    visual.update_image()
    root.mainloop()

main()
