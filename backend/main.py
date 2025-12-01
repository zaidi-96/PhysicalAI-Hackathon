from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(title="Physical AI Chatbot API")

from dotenv import load_dotenv
load_dotenv()  
GEMINI_API_KEY = os.getenv("AIzaSyDL-17JvQ3nw4lsmj3Nh-6EAcNbM_4TixA")
if not GEMINI_API_KEY:
    print("❌ ERROR: GEMINI_API_KEY not found in environment variables")
    print("Please set GEMINI_API_KEY in .env file or environment")
    exit(1)
# CORS setup
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure Gemini AI
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "AIzaSyDL-17JvQ3nw4lsmj3Nh-6EAcNbM_4TixA")
genai.configure(api_key=GEMINI_API_KEY)

# Model
model = genai.GenerativeModel('gemini-pro')

# Request model
class QuestionRequest(BaseModel):
    question: str
    context: str = None

@app.get("/")
def home():
    return {"message": "Physical AI Chatbot API", "status": "running"}

@app.post("/chat")
async def chat(request: QuestionRequest):
    try:
        # Create prompt
        if request.context:
            prompt = f"""
            Based on this context from the Physical AI textbook:
            
            CONTEXT:
            {request.context}
            
            QUESTION:
            {request.question}
            
            Answer the question using ONLY the context provided.
            If the answer is not in context, say "I cannot answer this based on the textbook."
            Keep answers concise and helpful for Pakistani students.
            """
        else:
            prompt = f"""
            You are an AI assistant for the "Physical AI & Humanoid Robotics" textbook.
            
            The textbook covers:
            1. ROS 2 - Robot Operating System
            2. Gazebo - Robotics Simulation
            3. NVIDIA Isaac - AI Robotics Platform
            4. VLM Models - Vision Language Models
            5. Capstone Project - Autonomous Humanoid
            
            QUESTION: {request.question}
            
            Answer in a helpful way for Pakistani students learning robotics.
            If you don't know, say you don't know.
            """
        
        # Generate response
        response = model.generate_content(prompt)
        
        return {
            "success": True,
            "answer": response.text,
            "question": request.question
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
def health():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)