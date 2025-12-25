---
sidebar_position: 2
title: "Voice Commands"
description: "Implementing voice-controlled robot systems with speech recognition and synthesis"
---

# Voice Commands

## Introduction

Voice command systems enable hands-free robot control through spoken instructions, combining speech recognition, command processing, and speech synthesis.

## System Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Microphone │───▶│     ASR     │───▶│   Command   │
│   Array     │    │   Engine    │    │   Parser    │
└─────────────┘    └─────────────┘    └──────┬──────┘
                                             │
┌─────────────┐    ┌─────────────┐    ┌──────▼──────┐
│   Speaker   │◀───│     TTS     │◀───│   Robot     │
│             │    │   Engine    │    │  Controller │
└─────────────┘    └─────────────┘    └─────────────┘
```

## Speech Recognition (ASR)

### Whisper (OpenAI)
```python
import whisper

model = whisper.load_model("base")

def transcribe_audio(audio_file):
    result = model.transcribe(audio_file)
    return result["text"]
```

### Google Speech-to-Text
```python
from google.cloud import speech

client = speech.SpeechClient()

def recognize_speech(audio_content):
    audio = speech.RecognitionAudio(content=audio_content)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="en-US",
    )
    response = client.recognize(config=config, audio=audio)
    return response.results[0].alternatives[0].transcript
```

### Vosk (Offline)
```python
from vosk import Model, KaldiRecognizer
import pyaudio

model = Model("model")
recognizer = KaldiRecognizer(model, 16000)

def listen_continuous():
    p = pyaudio.PyAudio()
    stream = p.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=8000
    )
    
    while True:
        data = stream.read(4000)
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            yield result
```

## Wake Word Detection

### Porcupine
```python
import pvporcupine

porcupine = pvporcupine.create(
    keywords=["robot", "hey robot"]
)

def detect_wake_word(audio_frame):
    keyword_index = porcupine.process(audio_frame)
    if keyword_index >= 0:
        return True
    return False
```

### Custom Wake Words
- Train on specific phrases
- Low false positive rate
- Always-on listening

## Command Processing

### Intent Recognition
```python
from transformers import pipeline

classifier = pipeline("zero-shot-classification")

def classify_intent(text):
    labels = ["navigation", "manipulation", "query", "stop"]
    result = classifier(text, labels)
    return result["labels"][0]
```

### Slot Filling
```python
def extract_slots(text, intent):
    slots = {}
    
    if intent == "navigation":
        # Extract location
        locations = ["kitchen", "bedroom", "office", "door"]
        for loc in locations:
            if loc in text.lower():
                slots["destination"] = loc
                break
    
    elif intent == "manipulation":
        # Extract object and action
        objects = ["cup", "ball", "bottle", "box"]
        for obj in objects:
            if obj in text.lower():
                slots["object"] = obj
                break
    
    return slots
```

## Text-to-Speech (TTS)

### pyttsx3 (Offline)
```python
import pyttsx3

engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)

def speak(text):
    engine.say(text)
    engine.runAndWait()
```

### Google TTS
```python
from gtts import gTTS
import pygame

def speak_gtts(text):
    tts = gTTS(text=text, lang='en')
    tts.save("response.mp3")
    
    pygame.mixer.init()
    pygame.mixer.music.load("response.mp3")
    pygame.mixer.music.play()
```

### ElevenLabs (High Quality)
```python
from elevenlabs import generate, play

def speak_elevenlabs(text):
    audio = generate(
        text=text,
        voice="Rachel",
        model="eleven_monolingual_v1"
    )
    play(audio)
```

## Complete Voice Command System

```python
class VoiceCommandSystem:
    def __init__(self):
        self.asr = whisper.load_model("base")
        self.tts = pyttsx3.init()
        self.listening = False
    
    def listen(self):
        """Record and transcribe audio"""
        audio = self.record_audio()
        text = self.asr.transcribe(audio)["text"]
        return text
    
    def process_command(self, text):
        """Parse and execute command"""
        intent = classify_intent(text)
        slots = extract_slots(text, intent)
        
        return self.execute(intent, slots)
    
    def execute(self, intent, slots):
        """Execute robot action"""
        if intent == "navigation":
            return f"Navigating to {slots.get('destination', 'unknown')}"
        elif intent == "manipulation":
            return f"Picking up {slots.get('object', 'object')}"
        elif intent == "stop":
            return "Stopping all actions"
        else:
            return "Command not understood"
    
    def respond(self, message):
        """Speak response"""
        self.tts.say(message)
        self.tts.runAndWait()
    
    def run(self):
        """Main loop"""
        self.respond("Voice command system ready")
        
        while True:
            text = self.listen()
            if text:
                response = self.process_command(text)
                self.respond(response)
```

## ROS2 Integration

### Voice Command Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(
            String, 'voice_commands', 10
        )
        self.timer = self.create_timer(0.1, self.listen_callback)
    
    def listen_callback(self):
        # Listen for voice commands
        command = self.voice_system.listen()
        if command:
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
```

## Best Practices

### Noise Handling
- Use microphone arrays
- Apply noise cancellation
- Implement voice activity detection

### Confirmation
- Repeat back commands
- Ask for confirmation on critical actions
- Provide status feedback

### Error Recovery
- "I didn't understand, please repeat"
- Suggest alternatives
- Graceful degradation

## Summary

Voice command systems make robot interaction natural and accessible, requiring careful integration of ASR, NLU, and TTS components.

---

*Next: [LLM Integration](./llm-integration.md)*
