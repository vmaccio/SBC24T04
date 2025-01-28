import speech_recognition as sr

def transcribe_wav(wav_file):
  """
  Transcribes the audio in the given WAV file to text.

  Args:
    wav_file: Path to the WAV file.

  Returns:
    The transcribed text, or an empty string if transcription fails.
  """
  try:
    r = sr.Recognizer()
    with sr.AudioFile(wav_file) as source:
      audio = r.record(source)
    text = r.recognize_google(audio)
    return text
  except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
    return ""
  except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
    return ""

# Example usage:
wav_file_path = "uploads/received_file.wav" 
transcribed_text = transcribe_wav(wav_file_path)
print(transcribed_text)
