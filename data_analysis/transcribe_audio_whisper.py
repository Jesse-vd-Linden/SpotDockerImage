import whisper

model = whisper.load_model("base")  # You can choose different model sizes
result = model.transcribe("C:\dev\SpotDockerImage\output_audio.wav")
print(result["text"])

output_file_path = './transcribed_text.txt'

# Writing the transcribed text to the file
with open(output_file_path, 'w') as file:
    file.write(result["text"])