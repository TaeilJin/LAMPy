import numpy as np
import matplotlib.animation as animation
import matplotlib.colors as colors
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class Vis():
    
    def __init__(self):
        print('visualize class')

    def plot_animation_env_pose(self, joints, targets, envs, \
                       parents, 
                       target_for=None, target_up=None, 
                       filename=None,fps=30, axis_scale=50, elev=45, azim=45):
        joints = joints.reshape((len(joints), -1, 3))
        
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-axis_scale, axis_scale)
        ax.set_zlim3d( 0, axis_scale)
        ax.set_ylim3d(-axis_scale, axis_scale)
        ax.grid(True)
        ax.set_axis_off()

        ax.view_init(elev=elev, azim=azim)

        xs = np.linspace(-200, 200, 50)
        ys = np.linspace(-200, 200, 50)
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros(X.shape)
        
        wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2, color='grey',lw=0.2)
        
        points = []
        lines = []

        forward_lines =[]
        up_lines = []

        acolors = list(sorted(colors.cnames.keys()))[::-1]
        lines.append([plt.plot([0,0], [0,0], [0,0], color='red', 
            lw=2, path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])[0] for _ in range(joints.shape[1])])
        
        if target_for is not None:
            forward_lines.append([plt.plot([0,0], [0,0], [0,0], color='yellow', 
                lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0] for _ in range(target_for.shape[1])]) # target forward
        if target_up is not None:
            up_lines.append([plt.plot([0,0], [0,0], [0,0], color='purple', 
                lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0] for _ in range(target_up.shape[1])]) # target up
        
        # Add points plotting
        points.append([ax.plot([], [], [], 'o', color='blue')[0] for _ in range(joints.shape[1])]) # 0 : target joints
        points.append([ax.plot([], [], [], 'o', color='green')[0] for _ in range(targets.shape[1])]) 
        points.append([ax.plot([], [], [], 'o', color='orange')[0] for _ in range(envs.shape[1])])
        
        # Text object for displaying frame number
        frame_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)
        def animate(i):
            
            changed = []
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[0][j].set_data(np.array([[joints[i,j,0], joints[i,parents[j],0]],[-joints[i,j,2],-joints[i,parents[j],2]]]))
                    lines[0][j].set_3d_properties(np.array([ joints[i,j,1],joints[i,parents[j],1]]))
            
            # Update points
            for j in range(joints.shape[1]):
                points[0][j].set_data(joints[i, j, 0], -joints[i, j, 2])
                points[0][j].set_3d_properties(joints[i, j, 1])
            
            for j in range(targets.shape[1]):
                points[1][j].set_data(targets[i,j, 0], -targets[i,j, 2])
                points[1][j].set_3d_properties(targets[i,j, 1])
            
            if target_for is not None:
                for j in range(target_for.shape[1]):
                    forward_lines[0][j].set_data(np.array([[targets[i,j,0], targets[i,j,0] + target_for[i,j,0]],[-targets[i,j,2],-targets[i,j,2] + -target_for[i,j,2]]]))
                    forward_lines[0][j].set_3d_properties(np.array([ targets[i,j,1],targets[i,j,1] + target_for[i,j,1]]))
            if target_up is not None:
                for j in range(target_up.shape[1]):
                    up_lines[0][j].set_data(np.array([[targets[i,j,0], targets[i,j,0] + target_up[i,j,0]],[-targets[i,j,2],-targets[i,j,2] + -target_up[i,j,2]]]))
                    up_lines[0][j].set_3d_properties(np.array([ targets[i,j,1],targets[i,j,1] + target_up[i,j,1]]))
            
            for j in range(envs.shape[1]):
                points[2][j].set_data(envs[i,j, 0], -envs[i,j, 2])
                points[2][j].set_3d_properties(envs[i,j, 1])

            # Update frame number text
            frame_text.set_text(f"Frame: {i}")

            changed += lines
            changed += points
            changed += forward_lines
            changed += up_lines
            changed.append(frame_text)
            return changed
            
        plt.tight_layout()
            
        ani = animation.FuncAnimation(fig, 
            animate, np.arange(joints.shape[0]), interval=1000/fps)

        if filename != None:
            #ani.save(filename, fps=fps, bitrate=13934)
            writergif = animation.PillowWriter(fps=30)
            ani.save(f'{filename}.gif',writer=writergif)
            
            ani.event_source.stop()
            del ani
            plt.close()    
        try:
            plt.show()
            plt.save()
        except AttributeError as e:
            pass
    

    def plot_animation_pose_wTarget(self, joints, targets, \
                       parents, filename=None,fps=30, axis_scale=50, elev=45, azim=45):
        joints = joints.reshape((len(joints), -1, 3))
        
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-axis_scale, axis_scale)
        ax.set_zlim3d( 0, axis_scale)
        ax.set_ylim3d(-axis_scale, axis_scale)
        ax.grid(True)
        ax.set_axis_off()

        ax.view_init(elev=elev, azim=azim)

        xs = np.linspace(-200, 200, 50)
        ys = np.linspace(-200, 200, 50)
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros(X.shape)
        
        wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2, color='grey',lw=0.2)
        
        points = []
        lines = []
        acolors = list(sorted(colors.cnames.keys()))[::-1]
        lines.append([plt.plot([0,0], [0,0], [0,0], color='red', 
            lw=2, path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])[0] for _ in range(joints.shape[1])])
        
        # Add points plotting
        points.append([ax.plot([], [], [], 'o', color='blue')[0] for _ in range(joints.shape[1])])
        
        points.append([ax.plot([], [], [], 'o', color='green')[0] for _ in range(targets.shape[1])])

        def animate(i):
            
            changed = []
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[0][j].set_data(np.array([[joints[i,j,0], joints[i,parents[j],0]],[-joints[i,j,2],-joints[i,parents[j],2]]]))
                    lines[0][j].set_3d_properties(np.array([ joints[i,j,1],joints[i,parents[j],1]]))
            
            # Update points
            for j in range(joints.shape[1]):
                points[0][j].set_data(joints[i, j, 0], -joints[i, j, 2])
                points[0][j].set_3d_properties(joints[i, j, 1])
            
            for j in range(targets.shape[1]):
                points[1][j].set_data(targets[i,j, 0], -targets[i,j, 2])
                points[1][j].set_3d_properties(targets[i,j, 1])

            changed += lines
            changed += points

            return changed
            
        plt.tight_layout()
            
        ani = animation.FuncAnimation(fig, 
            animate, np.arange(joints.shape[0]), interval=1000/fps)

        if filename != None:
            #ani.save(filename, fps=fps, bitrate=13934)
            writergif = animation.PillowWriter(fps=30)
            ani.save(f'{filename}.gif',writer=writergif)
            
            ani.event_source.stop()
            del ani
            plt.close()    
        try:
            plt.show()
            plt.save()
        except AttributeError as e:
            pass
        
    def plot_animation(self,joints, translations,\
                                parents, filename=None, fps=30, axis_scale=50, elev=45, azim=45):

        joints = joints.reshape((len(joints), -1, 3))
                
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-axis_scale, axis_scale)
        ax.set_zlim3d( 0, axis_scale)
        ax.set_ylim3d(-axis_scale, axis_scale)
        ax.grid(True)
        ax.set_axis_off()

        ax.view_init(elev=elev, azim=azim)

        xs = np.linspace(-200, 200, 50)
        ys = np.linspace(-200, 200, 50)
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros(X.shape)
        
        wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2, color='grey',lw=0.2)
        
        points = []
        lines = []
        acolors = list(sorted(colors.cnames.keys()))[::-1]
        tmp = np.zeros(translations.shape)    
        lines.append(plt.plot(translations[:,0],-translations[:,2], 
            lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0])
        lines.append([plt.plot([0,0], [0,0], [0,0], color='red', 
            lw=2, path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])[0] for _ in range(joints.shape[1])])
        
        def animate(i):
            
            changed = []
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[1][j].set_data(np.array([[joints[i,j,0], joints[i,parents[j],0]],[-joints[i,j,2],-joints[i,parents[j],2]]]))
                    lines[1][j].set_3d_properties(np.array([ joints[i,j,1],joints[i,parents[j],1]]))
            
            changed += lines
                
            return changed
            
        plt.tight_layout()
            
        ani = animation.FuncAnimation(fig, 
            animate, np.arange(joints.shape[0]), interval=1000/fps)

        if filename != None:
            #ani.save(filename, fps=fps, bitrate=13934)
            writergif = animation.PillowWriter(fps=30)
            ani.save(f'{filename}.gif',writer=writergif)
            
            ani.event_source.stop()
            del ani
            plt.close()    
        try:
            plt.show()
            plt.save()
        except AttributeError as e:
            pass

    def plot_animation_withRef(self,joints, joints_ref, root_vel,\
                                parents, filename=None, fps=30, axis_scale=50, elev=45, azim=45):

        rot = R.from_quat([0,0,0,1])
        translation = np.array([[0,0,0]])
        translations = np.zeros((root_vel.shape[0],3))
        
        root_dx, root_dz, root_dr = root_vel[:,-3], root_vel[:,-2], root_vel[:,-1]
        joints = joints.reshape((len(joints), -1, 3))
        joints_ref = joints_ref.reshape((len(joints_ref), -1, 3))
        
        for i in range(len(joints)):
                
            translation = translation + rot.apply(np.array([root_dx[i], 0, root_dz[i]]))
            rot = R.from_rotvec(np.array([0,-root_dr[i],0])) * rot

            joints[i,:,:] = rot.apply(joints[i])
            joints[i,:,0] = joints[i,:,0] + translation[0,0]
            joints[i,:,2] = joints[i,:,2] + translation[0,2]
            
            joints_ref[i,:,:] = rot.apply(joints_ref[i])
            joints_ref[i,:,0] = joints_ref[i,:,0] + translation[0,0]
            joints_ref[i,:,2] = joints_ref[i,:,2] + translation[0,2]
            
            translations[i,:] = translation
                
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-axis_scale, axis_scale)
        ax.set_zlim3d( 0, axis_scale)
        ax.set_ylim3d(-axis_scale, axis_scale)
        ax.grid(True)
        ax.set_axis_off()

        ax.view_init(elev=elev, azim=azim)

        xs = np.linspace(-200, 200, 50)
        ys = np.linspace(-200, 200, 50)
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros(X.shape)
        
        wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2, color='grey',lw=0.2)
        
        points = []
        lines = []
        acolors = list(sorted(colors.cnames.keys()))[::-1]
        tmp = np.zeros(translations.shape)    
        lines.append(plt.plot(translations[:,0],-translations[:,2], 
            lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0])
        lines.append([plt.plot([0,0], [0,0], [0,0], color='red', 
            lw=2, path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])[0] for _ in range(joints.shape[1])])
        lines.append([plt.plot([0,0], [0,0], [0,0], color='green', 
            lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0] for _ in range(joints_ref.shape[1])])
        def animate(i):
            
            changed = []
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[1][j].set_data(np.array([[joints[i,j,0], joints[i,parents[j],0]],[-joints[i,j,2],-joints[i,parents[j],2]]]))
                    lines[1][j].set_3d_properties(np.array([ joints[i,j,1],joints[i,parents[j],1]]))
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[2][j].set_data(np.array([[joints_ref[i,j,0], joints_ref[i,parents[j],0]],[-joints_ref[i,j,2],-joints_ref[i,parents[j],2]]]))
                    lines[2][j].set_3d_properties(np.array([ joints_ref[i,j,1],joints_ref[i,parents[j],1]]))

            changed += lines
                
            return changed
            
        plt.tight_layout()
            
        ani = animation.FuncAnimation(fig, 
            animate, np.arange(joints.shape[0]), interval=1000/fps)

        if filename != None:
            #ani.save(filename, fps=fps, bitrate=13934)
            writergif = animation.PillowWriter(fps=30)
            ani.save(f'{filename}.gif',writer=writergif)
            
            ani.event_source.stop()
            del ani
            plt.close()    
        try:
            plt.show()
            plt.save()
        except AttributeError as e:
            pass
    
    def plot_animation_withTR(self,joints, translations, \
                                parents, filename=None, fps=30, axis_scale=50, elev=45, azim=45):

        joints = joints.reshape((len(joints), -1, 3))
        
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-axis_scale, axis_scale)
        ax.set_zlim3d( 0, axis_scale)
        ax.set_ylim3d(-axis_scale, axis_scale)
        ax.grid(True)
        ax.set_axis_off()

        ax.view_init(elev=elev, azim=azim)

        xs = np.linspace(-200, 200, 50)
        ys = np.linspace(-200, 200, 50)
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros(X.shape)
        
        wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2, color='grey',lw=0.2)
        
        points = []
        lines = []
        acolors = list(sorted(colors.cnames.keys()))[::-1]
        tmp = np.zeros(translations.shape)    
        lines.append(plt.plot(translations[:,0],-translations[:,2], 
            lw=2, path_effects=[pe.Stroke(linewidth=1, foreground='black'), pe.Normal()])[0])
        lines.append([plt.plot([0,0], [0,0], [0,0], color='red', 
            lw=2, path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])[0] for _ in range(joints.shape[1])])
        
        def animate(i):
            
            changed = []
            
            for j in range(len(parents)):
                if parents[j] != -1:
                    lines[1][j].set_data(np.array([[joints[i,j,0], joints[i,parents[j],0]],[-joints[i,j,2],-joints[i,parents[j],2]]]))
                    lines[1][j].set_3d_properties(np.array([ joints[i,j,1],joints[i,parents[j],1]]))
            
            changed.append(points)

            changed += lines
            
            return changed
            
        plt.tight_layout()
            
        ani = animation.FuncAnimation(fig, 
            animate, np.arange(joints.shape[0]), interval=1000/fps)

        if filename != None:
            #ani.save(filename, fps=fps, bitrate=13934)
            writergif = animation.PillowWriter(fps=30)
            ani.save(f'{filename}.gif',writer=writergif)
            
            ani.event_source.stop()
            del ani
            plt.close()    
        try:
            plt.show()
            plt.save()
        except AttributeError as e:
            pass
