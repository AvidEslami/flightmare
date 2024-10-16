import seaborn as sns
import pandas as pd
import matplotlib.animation as animation
import matplotlib.pyplot as plt
#
sns.set_style("whitegrid")
#


def plot_reward(save_dir, varname, ylabel, save_fig=False):
    fig, ax = plt.subplots(1, 1, figsize=(7, 4))
    #
    sns.lineplot(data=data[varname].dropna(), ax=ax)
    plt.xlabel("Training Iterations")
    plt.title(ylabel + " (ppo)")
    if save_fig:
        plt.savefig(save_dir + "/" + ylabel + ".png")


if __name__ == "__main__":
    # logger_dir = "./saved/2024-09-21-17-54-51/"
    # logger_dir = "./saved/2024-09-29-19-36-48/"
    # logger_dir = "./saved/2024-10-01-18-58-01/" # Full Reward 6m
    # logger_dir = "./saved/2024-10-01-19-34-20/" # Full Reward - Orientation 6m
    # logger_dir = "./saved/2024-10-01-19-48-50/" # Full Reward - Orientation 6m -> relu not leaky, higher learning rate
    # logger_dir = "./saved/2024-10-01-20-05-34/" # Full Reward - Orientation 3m+6m -> relu not leaky, higher learning rate
    # logger_dir = "./saved/2024-10-01-20-25-26/" # Full Reward - Orientation 3m+6m -> relu not leaky, even higher learning rate
    # logger_dir = "./saved/2024-10-01-20-44-14/" # Full Reward 3m+6m -> relu not leaky, even higher learning rate
    # logger_dir = "./saved/2024-10-01-22-18-46/" # Big Circle

    # logger_dir = "./saved/2024-10-08-19-43-17/"  # New input scheme
    # logger_dir = "./saved/2024-10-08-21-12-02/"  # New input scheme, 5 lr, only see every 4th step
    # logger_dir = "./saved/2024-10-08-22-16-24/"  # New input scheme, 5 lr, only see every 4th step, fixed non relative pos...
    # logger_dir = "./saved/2024-10-09-00-28-57/"  # New input scheme, 5 lr, only see every 4th step, fixed non relative pos...

    # logger_dir = "./saved/2024-10-15-19-04-38/"  # New input scheme, 2 train hor, 0.5 view hor, pos,vel,ori reward
    logger_dir = "./saved/2024-10-15-20-13-57/"  # New input scheme, 2 train hor, 0.5 view hor, pos,vel,ori reward


    ppo_var_names = ["ep_reward_mean", "ep_len_mean", "policy_entropy"]
    ppo_y_labels = ["Reward", "Episode Length", "Policy Entropy"]
    #
    sac_var_names = ["train_reward", "test_reward", "entropy"]
    sac_y_labels = ["Train Reward", "Test Reward", "Entropy"]
    #
    csv_path = logger_dir + "progress.csv"
    data = pd.read_csv(csv_path)
    for i in range(len(ppo_var_names)):
        plot_reward(
            logger_dir, varname=ppo_var_names[i], ylabel=ppo_y_labels[i])
    plt.show()
