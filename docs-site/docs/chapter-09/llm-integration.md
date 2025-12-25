---
sidebar_position: 3
title: "LLM Integration"
description: "Integrating Large Language Models with robots for advanced reasoning and task planning"
---

# LLM Integration

## Introduction

Large Language Models (LLMs) bring powerful reasoning, planning, and communication capabilities to robots, enabling more sophisticated human-robot interaction and autonomous decision-making.

## Why LLMs for Robotics?

### Advanced Understanding
- Complex instruction interpretation
- Context-aware responses
- Multi-turn conversations

### Task Planning
- Breaking down complex tasks
- Generating action sequences
- Handling novel situations

### Knowledge Access
- World knowledge
- Common sense reasoning
- Object affordances

## LLM-Robot Architecture

```
┌─────────────────────────────────────────────────────┐
│                    User Input                        │
│            "Make me a sandwich"                      │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│                  LLM Planner                         │
│  - Understand intent                                 │
│  - Generate task plan                                │
│  - Handle constraints                                │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│              Skill/Action Library                    │
│  navigate(), pick(), place(), open(), close()       │
└────────────────────────┬────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────┐
│              Robot Execution                         │
│  Low-level control, sensor feedback                 │
└─────────────────────────────────────────────────────┘
```

## LLM Providers

### OpenAI GPT-4
```python
from openai import OpenAI

client = OpenAI()

def get_robot_plan(instruction):
    response = client.chat.completions.create(
        model="gpt-4-turbo",
        messages=[
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": instruction}
        ]
    )
    return response.choices[0].message.content
```

### Anthropic Claude
```python
from anthropic import Anthropic

client = Anthropic()

def get_robot_plan(instruction):
    response = client.messages.create(
        model="claude-3-sonnet-20240229",
        max_tokens=1024,
        system=ROBOT_SYSTEM_PROMPT,
        messages=[
            {"role": "user", "content": instruction}
        ]
    )
    return response.content[0].text
```

### Local Models (Ollama)
```python
import ollama

def get_robot_plan(instruction):
    response = ollama.chat(
        model='llama3',
        messages=[
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": instruction}
        ]
    )
    return response['message']['content']
```

## Prompting Strategies

### System Prompt for Robots
```python
ROBOT_SYSTEM_PROMPT = """
You are a robot assistant with the following capabilities:
- navigate(location): Move to a location
- pick(object): Grasp an object
- place(object, location): Put object at location
- open(object): Open a container/door
- close(object): Close a container/door
- speak(message): Say something to the user

Current environment:
- Kitchen with table, counter, fridge, cabinet
- Objects: cups, plates, bottles, food items

Respond with a sequence of actions in JSON format.
Always verify object visibility before manipulation.
Ask for clarification if the instruction is ambiguous.
"""
```

### Task Decomposition
```python
def decompose_task(instruction):
    prompt = f"""
    Decompose this task into robot-executable steps:
    Task: {instruction}
    
    Available actions: navigate, pick, place, open, close, speak
    
    Return JSON array of steps with action and parameters.
    """
    return get_robot_plan(prompt)

# Example output
# [
#   {"action": "navigate", "params": {"location": "kitchen"}},
#   {"action": "open", "params": {"object": "fridge"}},
#   {"action": "pick", "params": {"object": "water_bottle"}},
#   {"action": "close", "params": {"object": "fridge"}},
#   {"action": "navigate", "params": {"location": "user"}},
#   {"action": "speak", "params": {"message": "Here's your water"}}
# ]
```

## Code Generation for Robots

### SayCan Approach
```python
def saycan_planning(instruction, available_skills):
    """
    Combine LLM scoring with skill affordances
    """
    # Get LLM to score relevance of each skill
    skill_scores = {}
    for skill in available_skills:
        prompt = f"How relevant is '{skill}' for '{instruction}'? Score 0-1."
        score = float(get_robot_plan(prompt))
        skill_scores[skill] = score
    
    # Combine with affordance (can we actually do it?)
    for skill in available_skills:
        affordance = check_skill_affordance(skill)
        skill_scores[skill] *= affordance
    
    # Select highest scoring skill
    return max(skill_scores, key=skill_scores.get)
```

### Code as Policies
```python
def generate_robot_code(instruction):
    prompt = f"""
    Generate Python code for a robot to: {instruction}
    
    Available functions:
    - robot.move_to(x, y, z)
    - robot.gripper.open()
    - robot.gripper.close()
    - robot.get_object_position(name)
    - robot.is_holding()
    
    ```python
    """
    
    code = get_robot_plan(prompt)
    return code

# Generated code example:
# obj_pos = robot.get_object_position("cup")
# robot.move_to(obj_pos.x, obj_pos.y, obj_pos.z + 0.1)
# robot.gripper.open()
# robot.move_to(obj_pos.x, obj_pos.y, obj_pos.z)
# robot.gripper.close()
```

## Vision-Language Models

### GPT-4 Vision for Robotics
```python
import base64

def analyze_scene(image_path, question):
    with open(image_path, "rb") as f:
        image_data = base64.b64encode(f.read()).decode()
    
    response = client.chat.completions.create(
        model="gpt-4-vision-preview",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": question},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_data}"
                        }
                    }
                ]
            }
        ]
    )
    return response.choices[0].message.content

# Usage
scene_description = analyze_scene(
    "robot_camera.jpg",
    "What objects are on the table and where are they located?"
)
```

## Error Handling & Recovery

### LLM-Based Error Recovery
```python
def handle_execution_error(task, error, context):
    prompt = f"""
    Robot encountered an error:
    Task: {task}
    Error: {error}
    Context: {context}
    
    Suggest recovery action or alternative approach.
    """
    
    recovery_plan = get_robot_plan(prompt)
    return recovery_plan
```

## Safety Considerations

### Action Validation
```python
FORBIDDEN_ACTIONS = ["throw", "break", "hit", "harm"]

def validate_action(action):
    # Check against forbidden actions
    for forbidden in FORBIDDEN_ACTIONS:
        if forbidden in action.lower():
            return False, "Action not permitted for safety"
    
    # Check physical feasibility
    if not is_physically_possible(action):
        return False, "Action not physically feasible"
    
    return True, "Action validated"
```

### Human Confirmation
```python
def execute_with_confirmation(plan):
    for step in plan:
        if step.risk_level > THRESHOLD:
            speak(f"I'm about to {step.description}. Is that okay?")
            if not get_user_confirmation():
                return abort_plan()
        execute_step(step)
```

## Summary

LLM integration transforms robots from pre-programmed machines to flexible, reasoning agents capable of understanding complex instructions and adapting to new situations.

---

*Continue to Chapter 10: [Real-World Applications](../chapter-10/industrial-robotics.md)*
