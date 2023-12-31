import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns

data_speech = [
    {
        "Task": "Say command",
        "Start": 0,
        "Duration": 1000,
    },
    {
        "Task": "Speech Recognition Time",
        "Start": 1000,
        "Duration": 1300,
    },
    {
        "Task": "Timer for Audio Feedback",
        "Start": 2300,
        "Duration": 3000,
    },
    {
        "Task": "Recognition Feedback Panel",
        "Start": 2800,
        "Duration": 1500,
    },
    {
        "Task": "Robot Movement",
        "Start": 2800,
        "Duration": 2500,
    },
]

data_gestures= [
    {
        "Task": "Hold Gesture",
        "Start": 0,
        "Duration": 2200,
    },
    {
        "Task": "Timer for Audio Feedback",
        "Start": 1700,
        "Duration": 3000,
    },
    {
        "Task": "Recognition Feedback Panel",
        "Start": 2200,
        "Duration": 1500,
    },
    {
        "Task": "Robot Movement",
        "Start": 2200,
        "Duration": 2500,
    },
]


tasks_speech = len(data_speech)
tasks_gestures = len(data_gestures)

df = pd.DataFrame(data_speech)
df = df.iloc[::-1]

# Creating a figure
plt.figure(figsize=(10, 3))

cmap =  mpl.cm.get_cmap("tab20b")

# Plotting each task
for i, row in df.iterrows():
    plt.barh((tasks_speech - i - 1), row["Duration"], left=row["Start"], color=cmap((i / 5)+0.1))

    # Adding task label
    plt.text(row["Start"] + row["Duration"] / 2, (tasks_speech - i - 1), f'{row["Task"]}', color='black', ha="center", va="center")

# Setting labels and title
plt.xlabel("Time [ms]")
plt.ylabel("Tasks")
plt.title("Speech Recognition and Task Execution Timeline")

# Formatting x-axis to show milliseconds
plt.gca().xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{int(x)} ms'))

plt.tight_layout()
plt.show()


df = pd.DataFrame(data_gestures)
print(df)
df = df.iloc[::-1]
print(df)

# Creating a figure
plt.figure(figsize=(10, 3))

cmap =  mpl.cm.get_cmap("tab20b")

# Plotting each task
for i, row in df.iterrows():
    plt.barh((tasks_gestures - i - 1), row["Duration"], left=row["Start"], color=cmap((i / 5)+0.1))

    # Adding task label
    plt.text(row["Start"] + row["Duration"] / 2, (tasks_gestures - i - 1), f'{row["Task"]}', color='black', ha="center", va="center")

# Setting labels and title
plt.xlabel("Time [ms]")
plt.ylabel("Tasks")
plt.title("Gesture Recognition and Task Execution Timeline")

# Adjusting the x-axis limits
# plt.xlim(0, (df["Duration"].values[0] + df["Start"].values[0])+200)

# Formatting x-axis to show milliseconds
plt.gca().xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{int(x)} ms'))

plt.tight_layout()
plt.show()