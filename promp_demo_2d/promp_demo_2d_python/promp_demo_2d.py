#!/usr/bin/env python3.5
import matplotlib.pyplot as plt

import numpy as np
from pynput.keyboard import Key, Listener
import os, threading, pynput, random, stat, ast
from promp_context.promp_context import ProMPContext
from os import listdir
from os.path import isfile, join

class PrompDemo2D():
    def __init__(self):
        self.a = 0
        self.a_max = 2
        self.a_min = -2

        self.T = 4
        self.dt = 0.03
        self.t0 = 0
        self.t = np.arange(self.t0, self.T, self.dt)
        
        self.v = np.empty((self.t.shape))
        self.y = np.empty((self.t.shape))
        self.v[:] = np.NaN
        self.y[:] = np.NaN
        self.v[0] = 0
        self.y[0] = 0

        self.promp = ProMPContext(output_name=['y'], context_names=['y1', 'y2'], num_basis=20, num_samples=len(self.t))
        self.demonstrations = []

        self.figure = plt.figure()
        self.mode = 1

    def on_press(self, key):

        if key == Key.up:
            print("Pressed up")
            if self.mode == 1:
                self.a = 15
            elif self.mode == 0:
                self.a = 100
        elif key == Key.down:
            print("Pressed down")
            if self.mode == 1:
                self.a = -15
            elif self.mode == 0:
                self.a = -100

    def on_release(self, key):
        if key == Key.up:
            self.a = 0
        elif key == Key.down:
            self.a = 0
        
        elif key == Key.esc:
            # kill when esc is pressed
            os.system('kill %d' % os.getpid())
            raise Listener.StopException

    def get_demonstration_file_name(self, path):
        # get existing files from folder
        files = [f for f in listdir(path) if isfile(join(path, f))]
        
        # get numbers from these files
        numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in files]
        
        # sort them in ascending order
        numbers.sort()

        # make list of these files
        files = ["demonstration_" + str(number) + ".txt" for number in numbers]
        
        try:
            # add 1 to the last trajectory number and create new name
            return "demonstration_" + str(int(os.path.splitext(files[-1])[0].split('_')[-1]) + 1) + ".txt"
        except IndexError:
            # no files in folder yet --> create first file
            return "demonstration_1.txt"
    
    def save_data(self, path, file_name, demonstration):

        with open(path+file_name, 'w+') as f:
            f.write(str(demonstration))
        
        os.chmod(path+file_name,stat.S_IRWXO)
        os.chmod(path+file_name,stat.S_IRWXU)

        print("Saved demonstration data: " + str(file_name))

    def ode_update_step(self):
        

    def demonstrate(self, context, fig, canvas, draw_obstacle=False):
        # context1 = [2.0, random.randint(self.a_min, self.a_max)]
        # context2 = [3.6, random.randint(self.a_min, self.a_max)]
        context1 = [2.0, context[0]]
        context2 = [3.6, context[1]]

        t_plot = self.t[50]
        y_plot = -0.5

        
        plt.ion()
        # plt.figure(fig.number)
        for i in range(len(self.t)):
            plt.figure(fig.number)

            try:
                self.v[i+1] = float(self.v[i] + self.dt*self.a)
                self.y[i+1] = float(self.y[i] + self.dt*self.v[i+1])
                circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
                circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)
                obstacle = plt.Rectangle((t_plot, y_plot), 1, 1, linewidth=1, fill=True)


                plt.xlim( (self.t0, self.T) )
                plt.ylim( (self.a_min, self.a_max) )

                plt.plot(self.t[:i], self.y[:i])
                plt.grid()
                ax = plt.gca()
                # ax = fig.add_subplot(111)
                ax.plot(self.t[:i], self.y[:i])
                
                ax.set_aspect('equal')
                
                ax.add_artist(circle1)
                ax.add_artist(circle2)
                if draw_obstacle == 1:
                    ax.add_artist(obstacle)
                plt.draw()
        
                plt.pause(self.dt)
                plt.clf()

            except IndexError:
                plt.close()
        
        demonstration = ( list(self.y), [context1[1], context2[1]] )
        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/data/'
        file_name = self.get_demonstration_file_name(path)
        self.save_data(path, file_name, demonstration)
        
    def load_demonstration_from_folder(self, path, traj_file):
        with open(path+traj_file, "r") as traj:
            demo = ast.literal_eval(traj.read())    
            self.demonstrations.append(demo)
    
    def load_demonstrations_from_folder(self, path):
        files = [name for name in os.listdir(path) if os.path.isfile(os.path.join(path, name))]
        
        # get numbers from these files
        numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in files]
        
        # sort them in ascending order
        numbers.sort()

        # make list of these files
        files = ["demonstration_" + str(number) + ".txt" for number in numbers]

        for demo in files:
            self.load_demonstration_from_folder(path, demo)
    
    def build_model(self):
        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/data/'
        self.load_demonstrations_from_folder(path)

        for demo in self.demonstrations:
            self.promp.add_demonstration( (np.asarray([demo[0]]).T, demo[1] ) )

    def generalize(self, context):
        return self.promp.generate_trajectory(context) 

    def refine(self):

        continue_prediction = 1

        try:
            draw_obstacle = int(input("Do you want to draw an obstacle? 1/0: "))
        except ValueError:
            draw_obstacle = 1

        while continue_prediction == 1:
            plt.switch_backend('TkAgg')

            context1 = [2.0, random.randrange(self.a_min+1, self.a_max-1, 1)]
            context2 = [3.6, random.randrange(self.a_min+1, self.a_max-1, 1)]
            context = [context1[1], context2[1]]

            self.y = np.asarray(self.generalize(context))
            t_plot = self.t[50]
            y_plot = -0.5

            circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
            circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)
            obstacle = plt.Rectangle((t_plot, y_plot), 1, 1, linewidth=1, fill=True)
            
            plt.xlabel("x")
            plt.ylabel("y")


            plt.xlim( (self.t0, self.T) )
            plt.ylim( (self.a_min, self.a_max) )

            plt.plot(self.t, self.y)
            ax = plt.gca()
            ax.set_aspect('equal')
            
            ax.add_artist(circle1)
            ax.add_artist(circle2)
            if draw_obstacle == 1:
                ax.add_artist(obstacle)
                
            plt.grid()
            plt.show()

            try:
                refinement_mode = int(input("Refine motion? 1/0: "))
            except ValueError: 
                refinement_mode = 1

            while refinement_mode == 1:
                self.v = np.empty((self.t.shape))
                self.v[:] = np.NaN
                self.v[0] = 0
            
                # plt.switch_backend('TkAgg')

                for i in range(len(self.t)):

                    try:
                        self.v[i+1] = self.v[i] + self.dt*self.a
                        self.y[i] = self.y[i] + self.dt*self.v[i+1]

                        plt.xlim( (self.t0, self.T) )
                        plt.ylim( (self.a_min, self.a_max) )
                    
                        circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
                        circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)
                        
                        obstacle = plt.Rectangle((t_plot, y_plot), 1, 1, linewidth=1, fill=True)

                        plt.plot(self.t, self.y, color='orange')
                        plt.plot(self.t[:i], self.y[:i], color='blue')
                        plt.xlabel("x")
                        plt.ylabel("y")
                        plt.grid()
                        plt.draw()
                        ax = plt.gca()
                        ax.set_aspect('equal')
                        
                        ax.add_artist(circle1)
                        ax.add_artist(circle2)
                        
                        if draw_obstacle == 1:
                            ax.add_artist(obstacle)

                        plt.pause(self.dt)
                        plt.clf()

                    except IndexError:
                        plt.close()
                try:
                    refinement_mode = int(input("Continue refining? 1/0: "))
                except ValueError: 
                    refinement_mode = 1

            demonstration = ( list(self.y), [context1[1], context2[1]] )
            
            try:
                amount = int(input("How many times do you want to add this demonstration to the model?: "))
            except ValueError:
                amount = 1

            for i in range(amount):
                self.promp.welford_update((np.asarray([demonstration[0]]).T, demonstration[1] ))

            try:
                continue_prediction = int(input("Do you want to continue predicting and refining new trajectories? 1/0: "))
            except ValueError:
                continue_prediction = 1
    
    def enable_keyboard(self):
        print("Enabled keyboard")
        with Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
                listener.join()

    def run(self):
        # try:
        #     self.mode = int(input("Teaching or refining?: 1/0"))
        # except:
        #     self.mode = 1

        # self.build_model()

        # if self.mode == 0:
        #     refine_thread = threading.Thread(target=self.refine, args=())
        #     refine_thread.start()
        # elif self.mode == 1:
        #     teaching_thread = threading.Thread(target=self.demonstrate, args=())
        #     teaching_thread.start()
        
        with Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
                listener.join()

        ###### GENERALIZE #######
        # context1 = [2.0, random.randrange(self.a_min, self.a_max, 2)]
        # context2 = [3.6, random.randrange(self.a_min, self.a_max, 2)]

        # plt.xlim( (self.t0, self.T) )
        # plt.ylim( (self.a_min, self.a_max) )
    
        # circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
        # circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)

        # plt.switch_backend('TkAgg')

        # context = [context1[1], context2[1]]
        # y = self.generalize(context)
        # plt.xlim( (self.t0, self.T) )
        # plt.ylim( (self.a_min, self.a_max) )

        # plt.plot(self.t, y)
        # ax = plt.gca()
        # ax.set_aspect('equal')
        
        # ax.add_artist(circle1)
        # ax.add_artist(circle2)
        # plt.grid()
        # plt.show()


if __name__ == "__main__":
    promp_demo_2d = PrompDemo2D()
    promp_demo_2d.run()