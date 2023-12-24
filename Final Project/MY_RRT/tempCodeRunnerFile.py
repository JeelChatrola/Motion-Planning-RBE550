def animation_global(self, nodelist, paths, title, save_filename='animation.gif'):
        self.plot_grid(title)

        script_dir = os.path.dirname(os.path.realpath(__file__))
        save_path = os.path.join(script_dir, save_filename)
        
        fig, ax = plt.subplots()
        ims = []
        
        for path in paths:
            # for nodes in nodelist:
            #     for node in nodes:
            #         if node.parent:
            #             plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='white')
            
            # Plot the sub-path
            x_values, y_values = zip(*path)
            im, = ax.plot(x_values, y_values, color='green', linewidth=1.5)
            ims.append([im])

            # plt.plot(x_values, y_values, color='green', linewidth=1.5)
            # plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
            # plt.pause(1)
        
        ani = animation.ArtistAnimation(fig, ims, interval=100, blit=True)
        ani.save(save_path, writer='imagemagick')
        plt.ioff()
        plt.show()
