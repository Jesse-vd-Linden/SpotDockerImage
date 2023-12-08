from google.cloud import speech

def transcribe_audio(speech_file):
    """Transcribes an audio file using Google Cloud Speech-to-Text."""

    client = speech.SpeechClient()

    with open(speech_file, "rb") as audio_file:
        content = audio_file.read()

    audio = speech.RecognitionAudio(content=content)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="en-US",
        enable_word_time_offsets=True,  # To get time stamps
        enable_automatic_punctuation=True,
        diarization_config=speech.SpeakerDiarizationConfig(
            enable_speaker_diarization=True,
            min_speaker_count=2,
            max_speaker_count=2
        )
    )

    response = client.recognize(config=config, audio=audio)

    for result in response.results:
        print("Transcript: {}".format(result.alternatives[0].transcript))
        print("Confidence: {}".format(result.alternatives[0].confidence))
        for word_info in result.alternatives[0].words:
            word = word_info.word
            start_time = word_info.start_time
            end_time = word_info.end_time
            print(f"Word: {word}, start_time: {start_time.total_seconds()}, end_time: {end_time.total_seconds()}")

transcribe_audio("C:\dev\SpotDockerImage\output_audio.wav")