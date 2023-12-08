from moviepy.editor import VideoFileClip

# Path to your video file
video_file = "C:\\Users\\jesse\\Downloads\\P001.mp4"

# Load the video file
video_clip = VideoFileClip(video_file)

# Extract the audio
audio_clip = video_clip.audio

# Path for the output audio file
audio_file = "output_audio.wav"

# Write the audio to a file
audio_clip.write_audiofile(audio_file)