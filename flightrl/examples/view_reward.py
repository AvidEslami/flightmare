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
    # logger_dir = "./saved/2024-10-15-20-13-57/"  # New input scheme, 2 train hor, 0.5 view hor, pos,vel,ori reward
    # logger_dir = "./saved/2024-10-15-23-37-00/"  # New input scheme, 2 train hor, 0.5 view hor, pos,vel,ori reward

    # logger_dir = "./saved/2024-10-20-12-36-09/" # Dummy Circle Path, *new input scheme same reward as before 0.5,2.0, no act penalty
    # logger_dir = "./saved/2024-10-20-13-44-52/" # Dummy Circle Path, *new input scheme same reward as before 0.5,2.0, basic act penalty, 4000 steps
    # logger_dir = "./saved/2024-10-20-15-05-34/" # DCP, act penalty relative to hover not normalized, 5000 steps 0.5,2.5, still spinning
    # logger_dir = "./saved/2024-10-20-15-54-24/" # Same as above but with a fixed normalized penalty
    # logger_dir = "./saved/2024-10-20-16-42-52/" # Same as above but with much larger penalty
    # logger_dir = "./saved/2024-10-20-19-00-36/" # Try again but with penalty hopefully large enough to just force hover command

    # logger_dir = "./saved/2024-10-21-20-11-32/"  # Dummy Circle Path, just pose as input, pos traj + act rewards
    # logger_dir = "./saved/2024-10-21-22-41-41/" # Fixed Hover act penalty, weird breaking points and jumps in training curves
    # logger_dir = "./saved/2024-10-22-00-01-08/" # Same as above but with 10000 steps instead of 6000 and 0.5,4.0 instead of 0.5,3.5 -> pos/vel*10, act*100
    # logger_dir = "./saved/2024-10-22-09-53-22/" # Same as above but with 24000 steps, for some reason ep_len_mean was 49 instead of 74
    # logger_dir = "./saved/2024-10-22-19-00-55/" # Same as above but with nminibatches set to 2 instead of 1
    # logger_dir = "./saved/2024-10-22-21-00-58/" # Same as 10000 steps case except only 2000 iterations and 10* act pen instead of 100*
    # logger_dir = "./saved/2024-10-22-22-25-19/" # Above but 4000 iterations, 5* vel pen instead of 10*
    # logger_dir = "./saved/2024-10-23-01-00-14/" # Tried above but 10000 iterations broke down

    # logger_dir = "./saved/2024-10-29-19-18-12/" # Same as above but with softplus instead of relu, training was stable, but spinning is back
    # logger_dir = "./saved/2024-10-29-19-18-12/" # Same as above but with more steps again? Drone wasn't spinning by the end of the training
    # logger_dir = "./saved/2024-10-29-20-54-11/" # 13530 steps, still softplus instead of relu, destruction became recoverable

    # logger_dir = "./saved/2024-11-02-18-39-38/" # 1400 steps, same as above, for reward -4 flight path is relatively good
    # logger_dir = "./saved/2024-11-03-15-13-17/" # Tried to replicate old setup, bad reward
    logger_dir = "./saved/2024-11-03-16-01-42/" # Tried again

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
