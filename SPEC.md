# Technical Specification: AI-Native Educational Platform
## Physical AI & Humanoid Robotics Textbook with RAG Chatbot

**Project**: GIAIC Hackathon - AI-Native Educational App  
**Date**: December 24, 2025  
**Version**: 1.0  
**Plan Reference**: [specs/master/plan.md](specs/master/plan.md)

## Executive Summary

A comprehensive educational platform combining a static Docusaurus-based textbook on Physical AI & Humanoid Robotics with an intelligent RAG (Retrieval-Augmented Generation) chatbot. The system embeds all textbook content into a vector database (Qdrant) and provides contextually accurate answers exclusively from the educational material, ensuring students receive grounded, curriculum-aligned responses.

### Core Features
- **Multi-page Textbook**: 50-100 pages across 10-15 chapters covering Physical AI and Humanoid Robotics
- **Intelligent Chatbot**: RAG-powered assistant that answers only from textbook content
- **Vector Search**: Semantic search using embedded textbook chunks (Qdrant vector database)
- **Citation System**: Answers include references to specific textbook sections
- **Responsive Design**: Optimized for desktop and mobile learning
- **Deployment**: Vercel (frontend) + Cloud deployment (backend)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         User Browser                                 │
│  - Reads textbook content                                           │
│  - Interacts with embedded chatbot                                  │
└──────────────┬────────────────────────────┬─────────────────────────┘
               │                            │
               │ (Static Content)           │ (REST API - HTTPS)
               │ Textbook Pages             │ POST /api/chat
               │ Images, Diagrams           │ GET /api/health
               ▼                            ▼
┌──────────────────────────┐    ┌──────────────────────────────────────┐
│  Docusaurus Frontend     │    │   FastAPI Backend                    │
│  (Vercel/Netlify)        │    │   (Railway/Render/DigitalOcean)      │
│                          │    │                                      │
│  ├─ docs/               │    │  ├─ Chat Router (/api/chat)          │
│  │  ├─ Intro           │    │  ├─ RAG Pipeline                     │
│  │  ├─ Physical AI     │    │  ├─ Embedding Service                │
│  │  ├─ Humanoid        │    │  ├─ LLM Integration                  │
│  │  └─ Robotics        │    │  └─ Response Formatter               │
│  ├─ src/               │    │                                      │
│  │  └─ ChatWidget.tsx  │◄───┤  CORS: Docusaurus domain            │
│  └─ static/            │    └──────────────┬───────────────────────┘
│     └─ img/            │                   │
└──────────────────────────┘                   │ Qdrant Python Client
                                               │ Async queries
                                               ▼
                                ┌──────────────────────────────────────┐
                                │  Qdrant Vector Database              │
                                │  (Qdrant Cloud or Self-hosted)       │
                                │                                      │
                                │  Collection: textbook_embeddings     │
                                │  ├─ Vectors (384-dim)                │
                                │  ├─ Payloads (text, metadata)        │
                                │  └─ Index (HNSW algorithm)           │
                                │                                      │
                                │  Metadata per chunk:                 │
                                │  - chapter_title                     │
                                │  - section_title                     │
                                │  - page_url                          │
                                │  - chunk_index                       │
                                └──────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│  Embedding Pipeline (One-time + Updates)                            │
│                                                                      │
│  Markdown Files → Parser → Chunker → Embedding Model →              │
│  → Vector DB Uploader                                               │
│                                                                      │
│  Triggered by: python scripts/ingest_docs.py                        │
└──────────────────────────────────────────────────────────────────────┘
```

### Data Flow: User Question → Answer

1. **User Input**: Student asks question via ChatWidget component
2. **API Request**: Frontend sends POST to `/api/chat` with question text
3. **Embedding Generation**: Backend embeds question using same model as textbook
4. **Vector Search**: Query Qdrant for top-k (5-10) most similar chunks
5. **Context Assembly**: Retrieved chunks combined with metadata
6. **LLM Prompt**: Strict system prompt + context + question sent to GPT-4/Claude
7. **Response Generation**: LLM generates answer constrained to provided context
8. **Citation Addition**: Backend adds source references (chapter/section URLs)
9. **API Response**: JSON with answer + citations sent to frontend
10. **UI Display**: ChatWidget renders answer with clickable citation links

---

## Technology Stack

### Frontend Stack

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Framework | Docusaurus | 3.1+ | Static site generation, documentation framework |
| UI Library | React | 18.x | Component library (Docusaurus dependency) |
| Language | TypeScript | 5.x | Type-safe development |
| Styling | CSS Modules | - | Component-scoped styling |
| State Management | React Hooks | - | Local state (useState, useEffect) |
| HTTP Client | Fetch API | Native | API calls to backend |
| Build Tool | Webpack | 5.x | Bundling (Docusaurus integrated) |
| Deployment | Vercel/Netlify | - | Static site hosting with CDN |

### Backend Stack

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Framework | FastAPI | 0.104+ | Async Python web framework |
| Language | Python | 3.11+ | Backend implementation |
| Vector Database | Qdrant | 1.7+ | Semantic search, embeddings storage |
| Embedding Model | all-MiniLM-L6-v2 | Latest | sentence-transformers model (384-dim) |
| LLM Provider | OpenAI GPT-4 Turbo | Latest | Response generation |
| RAG Framework | LangChain | 0.1+ | RAG pipeline orchestration |
| Data Validation | Pydantic | 2.5+ | Request/response schemas |
| ASGI Server | Uvicorn | 0.27+ | Production server |
| Deployment | Railway/Render | - | Container hosting |

### Alternative Technologies (Optional)

| Purpose | Primary Choice | Alternative 1 | Alternative 2 |
|---------|---------------|---------------|---------------|
| Vector DB | Qdrant (open-source) | Pinecone (managed) | Weaviate |
| Embedding | all-MiniLM-L6-v2 | OpenAI text-embedding-3-small | BGE-small-en |
| LLM | GPT-4 Turbo | Claude 3.5 Sonnet | Llama 2 70B (Ollama) |
| Backend Hosting | Railway | Render | DigitalOcean App Platform |
| Frontend Hosting | Vercel | Netlify | GitHub Pages |

---

## Folder Structure

```
my-project/
├── docs/                          # Docusaurus documentation site
│   ├── .docusaurus/              # Build cache (gitignored)
│   ├── build/                    # Production build (gitignored)
│   ├── docs/                     # Textbook markdown files
│   │   ├── intro.md
│   │   ├── chapter-01/
│   │   │   ├── index.md
│   │   │   ├── section-1.md
│   │   │   └── section-2.md
│   │   ├── chapter-02/
│   │   └── ...
│   ├── src/
│   │   ├── components/           # React components
│   │   │   ├── ChatWidget/
│   │   │   │   ├── index.tsx
│   │   │   │   └── styles.module.css
│   │   │   └── TTSButton/
│   │   ├── css/
│   │   │   └── custom.css
│   │   └── pages/
│   │       └── index.tsx
│   ├── static/
│   │   └── img/
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   ├── package.json
│   └── tsconfig.json
│
├── backend/                       # FastAPI backend
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py               # FastAPI app entry point
│   │   ├── config.py             # Configuration management
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   └── schemas.py        # Pydantic models
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── embeddings.py     # Embedding generation
│   │   │   ├── vector_store.py   # Qdrant/Pinecone client
│   │   │   ├── rag.py            # RAG pipeline
│   │   │   ├── tts.py            # Text-to-speech (optional)
│   │   │   └── auth.py           # Authentication (optional)
│   │   ├── routers/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py           # Chat endpoint
│   │   │   └── health.py         # Health check
│   │   └── utils/
│   │       ├── __init__.py
│   │       └── logger.py
│   ├── data/
│   │   └── textbook_markdown/    # Copy of markdown files
│   ├── scripts/
│   │   ├── ingest_documents.py   # Embed and upload to vector DB
│   │   └── test_rag.py           # RAG testing script
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── test_chat.py
│   │   └── test_embeddings.py
│   ├── .env.example
│   ├── requirements.txt
│   ├── Dockerfile                # For HF Spaces deployment
│   └── README.md
│
├── specs/                         # Spec-Kit specifications
│   ├── architecture.md
│   ├── frontend-spec.md
│   ├── backend-spec.md
│   ├── rag-spec.md
│   └── deployment-spec.md
│
├── demos/                         # Optional ROS2/Gazebo demos
│   ├── ros2-examples/
│   ├── gazebo-worlds/
│   └── isaac-sim-scenes/
│
├── .github/
│   └── workflows/
│       ├── frontend-deploy.yml   # Vercel deployment
│       └── backend-tests.yml
│
├── .gitignore
├── README.md
├── CLAUDE.md
├── CONSTITUTION.md
└── SPEC.md                        # This file
```

---

## Environment Variables

### Frontend (Docusaurus)

Create `docs/.env.local`:

```bash
# Backend API URL
REACT_APP_BACKEND_URL=https://your-space.hf.space
# or during development: http://localhost:8000

# Optional: Analytics
REACT_APP_GA_TRACKING_ID=G-XXXXXXXXXX
```

### Backend (FastAPI)

Create `backend/.env`:

```bash
# ===== Vector Database =====
# For Qdrant (local or cloud)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=textbook_chunks

# OR for Pinecone
PINECONE_API_KEY=your_pinecone_api_key
PINECONE_ENVIRONMENT=us-east-1-aws
PINECONE_INDEX_NAME=textbook-index

# ===== Embeddings Provider =====
# Option 1: OpenAI
OPENAI_API_KEY=sk-your-openai-api-key
EMBEDDING_MODEL=text-embedding-3-small

# Option 2: Hugging Face (for open-source models)
HF_TOKEN=hf_your_token
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

# ===== LLM Provider =====
# For OpenAI
OPENAI_API_KEY=sk-your-openai-api-key
LLM_MODEL=gpt-4-turbo-preview

# OR for Claude
ANTHROPIC_API_KEY=sk-ant-your-claude-key
LLM_MODEL=claude-3-5-sonnet-20241022

# ===== Application Settings =====
APP_ENV=production  # or development
LOG_LEVEL=INFO
CORS_ORIGINS=https://your-docusaurus-site.vercel.app,http://localhost:3000

# ===== Optional Features =====
# Text-to-Speech
TTS_ENABLED=true
TTS_PROVIDER=openai  # or elevenlabs, google
ELEVENLABS_API_KEY=your_elevenlabs_key

# Urdu Support
URDU_ENABLED=true
URDU_EMBEDDING_MODEL=sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2

# User Authentication
AUTH_ENABLED=false
JWT_SECRET=your-super-secret-jwt-key
JWT_ALGORITHM=HS256
JWT_EXPIRATION_MINUTES=1440

# ===== Hugging Face Spaces =====
HF_SPACE_NAME=your-username/textbook-backend
```

---

## Build and Run Steps

### 1. Frontend Setup (Docusaurus)

#### Initial Setup
```bash
cd docs

# Install dependencies
npm install

# Or create new Docusaurus site (if starting fresh)
npx create-docusaurus@latest docs classic --typescript
```

#### Development
```bash
# Start dev server (http://localhost:3000)
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

#### Environment Setup
```bash
# Create environment file
cp .env.example .env.local

# Edit .env.local with your backend URL
```

### 2. Backend Setup (FastAPI)

#### Initial Setup
```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

#### Create requirements.txt
```txt
fastapi==0.109.0
uvicorn[standard]==0.27.0
python-dotenv==1.0.0
pydantic==2.5.3
pydantic-settings==2.1.0

# Vector Database
qdrant-client==1.7.0
# pinecone-client==3.0.0  # Alternative

# Embeddings & LLM
openai==1.10.0
anthropic==0.18.0
langchain==0.1.6
langchain-openai==0.0.5
langchain-community==0.0.19

# Text Processing
sentence-transformers==2.3.1
tiktoken==0.5.2
pypdf==4.0.0
markdown==3.5.2

# Optional: TTS
elevenlabs==0.2.27

# Optional: Auth
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6

# Testing
pytest==8.0.0
httpx==0.26.0
```

#### Development
```bash
# Start FastAPI server (http://localhost:8000)
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Run with environment variables
python -m uvicorn app.main:app --reload
```

#### Ingest Textbook Data
```bash
# Copy markdown files from docs to backend
python scripts/ingest_documents.py

# This will:
# 1. Read all markdown files
# 2. Chunk them into smaller pieces
# 3. Generate embeddings
# 4. Upload to vector database
```

#### Testing
```bash
# Run tests
pytest

# Test RAG pipeline
python scripts/test_rag.py

# Test chat endpoint
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS2?", "language": "en"}'
```

### 3. Vector Database Setup

#### Option A: Qdrant (Local)
```bash
# Run Qdrant with Docker
docker run -p 6333:6333 -p 6334:6334 \
  -v $(pwd)/qdrant_storage:/qdrant/storage \
  qdrant/qdrant

# Or use Qdrant Cloud (managed service)
# Sign up at https://cloud.qdrant.io
```

#### Option B: Pinecone (Cloud)
```bash
# Sign up at https://www.pinecone.io
# Create an index via dashboard or API
# Use API key in .env file
```

---

## Deployment

### Frontend Deployment (Vercel)

#### Method 1: Vercel Dashboard
1. Push code to GitHub
2. Go to https://vercel.com/new
3. Import your repository
4. Set root directory to `docs`
5. Framework preset: Docusaurus
6. Add environment variables from `.env.local`
7. Deploy

#### Method 2: Vercel CLI
```bash
cd docs

# Install Vercel CLI
npm i -g vercel

# Login
vercel login

# Deploy
vercel

# Production deployment
vercel --prod
```

#### vercel.json Configuration
Create `docs/vercel.json`:
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "env": {
    "REACT_APP_BACKEND_URL": "@backend-url"
  }
}
```

### Backend Deployment (Hugging Face Spaces)

#### Setup Hugging Face Space
1. Create account at https://huggingface.co
2. Create new Space: https://huggingface.co/new-space
3. Choose "Docker" SDK (for FastAPI)
4. Clone the space repository

#### Dockerfile for HF Spaces
Create `backend/Dockerfile`:
```dockerfile
FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY app ./app
COPY data ./data
COPY scripts ./scripts

# Expose port
EXPOSE 7860

# Run FastAPI (HF Spaces uses port 7860)
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

#### HF Spaces Configuration
Create `backend/README.md` (HF Spaces metadata):
```markdown
---
title: Textbook RAG Backend
emoji: 📚
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
license: mit
---

# Textbook RAG Backend

FastAPI backend with RAG system for textbook Q&A.
```

#### Deploy to HF Spaces
```bash
# Clone your space
git clone https://huggingface.co/spaces/your-username/textbook-backend
cd textbook-backend

# Copy backend files
cp -r ../backend/* .

# Add secrets in HF Space settings dashboard:
# OPENAI_API_KEY, QDRANT_API_KEY, etc.

# Push to HF
git add .
git commit -m "Initial deployment"
git push
```

#### Environment Variables in HF Spaces
1. Go to Space Settings
2. Add secrets:
   - `OPENAI_API_KEY`
   - `QDRANT_API_KEY`
   - `QDRANT_URL`
   - `ANTHROPIC_API_KEY`
   - etc.

---

## Spec-Driven Workflow with AI Agents

### Using Spec-Kit

1. **Install Spec-Kit**
```bash
npm install -g spec-kit
# or
pip install spec-kit
```

2. **Generate Initial Specs**
```bash
spec-kit init

# Generate component specs
spec-kit generate component ChatWidget --framework react
spec-kit generate api chat --framework fastapi
```

3. **Refine with AI Agents**
```bash
# Use Claude API
export ANTHROPIC_API_KEY=your-key
spec-kit refine specs/frontend-spec.md --agent claude

# Or OpenAI
export OPENAI_API_KEY=your-key
spec-kit refine specs/backend-spec.md --agent openai
```

### Development Workflow

```
┌─────────────────────────────────────────────────────────┐
│ 1. Write Specification (specs/*.md)                     │
│    - Define requirements, API contracts, data models    │
└───────────────┬─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────┐
│ 2. AI Agent Review & Refinement                         │
│    - spec-kit refine with Claude/OpenAI                 │
│    - Generate test cases                                │
└───────────────┬─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Code Generation                                       │
│    - Use AI agents to scaffold components               │
│    - Generate boilerplate from specs                    │
└───────────────┬─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Implementation & Testing                              │
│    - Manual refinement                                   │
│    - Run tests, iterate                                  │
└───────────────┬─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────┐
│ 5. Deployment                                            │
│    - Frontend to Vercel                                  │
│    - Backend to HF Spaces                                │
└─────────────────────────────────────────────────────────┘
```

---

## Key Implementation Details

### RAG Pipeline (backend/app/services/rag.py)

```python
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

class RAGPipeline:
    def __init__(self):
        self.embeddings = OpenAIEmbeddings(model="text-embedding-3-small")
        self.qdrant = QdrantClient(url=os.getenv("QDRANT_URL"))
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME")
        
    def ingest_documents(self, markdown_files):
        """Chunk, embed, and upload documents"""
        # 1. Chunk documents
        # 2. Generate embeddings
        # 3. Upload to Qdrant
        
    def query(self, question: str, top_k: int = 5):
        """Retrieve relevant chunks and generate answer"""
        # 1. Embed question
        # 2. Search vector DB
        # 3. Pass to LLM with context
        # 4. Return answer
```

### Chat Widget (docs/src/components/ChatWidget/index.tsx)

```typescript
import React, { useState } from 'react';

export default function ChatWidget() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  
  const sendMessage = async () => {
    const response = await fetch(
      `${process.env.REACT_APP_BACKEND_URL}/chat`,
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 
          question: input,
          language: 'en' 
        })
      }
    );
    
    const data = await response.json();
    setMessages([...messages, { user: input, bot: data.answer }]);
  };
  
  return (
    <div className="chat-widget">
      {/* Chat UI */}
    </div>
  );
}
```

---

## Optional Features

### 1. Text-to-Speech (TTS)

**Backend Service** (backend/app/services/tts.py):
```python
from elevenlabs import generate, set_api_key

class TTSService:
    def __init__(self):
        set_api_key(os.getenv("ELEVENLABS_API_KEY"))
    
    def generate_audio(self, text: str, language: str = "en"):
        audio = generate(text=text, voice="Bella")
        return audio
```

**Frontend Component**:
```typescript
const playTTS = async (text: string) => {
  const response = await fetch(`${API_URL}/tts`, {
    method: 'POST',
    body: JSON.stringify({ text, language: 'en' })
  });
  const audioBlob = await response.blob();
  const audio = new Audio(URL.createObjectURL(audioBlob));
  audio.play();
};
```

### 2. Urdu Mode

- Use multilingual embeddings: `paraphrase-multilingual-MiniLM-L12-v2`
- Detect language in chat request
- Store separate Urdu markdown files in `docs/docs/ur/`
- Switch Docusaurus locale

### 3. User Authentication

**Backend** (backend/app/services/auth.py):
```python
from jose import JWTError, jwt
from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"])

def create_access_token(data: dict):
    # Generate JWT token
    
def verify_token(token: str):
    # Verify JWT
```

**Endpoints**:
- POST `/auth/register`
- POST `/auth/login`
- GET `/auth/me`

### 4. ROS2/Gazebo/Isaac Sim Demos

Store in `demos/` folder:
- **ROS2**: Launch files, node examples
- **Gazebo**: World files (.world), robot models (URDF)
- **Isaac Sim**: USD scenes, Python scripts

Embed in Docusaurus with:
```markdown
## ROS2 Example

<iframe src="/demos/ros2-viz.html" width="100%" height="600px"></iframe>

[Download Launch File](demos/ros2-examples/navigation.launch.py)
```

---

## Testing Checklist

- [ ] Frontend builds without errors
- [ ] Backend starts and serves /docs
- [ ] Document ingestion completes
- [ ] Chat endpoint returns relevant answers
- [ ] Answers are grounded in textbook content only
- [ ] CORS configured correctly
- [ ] Environment variables loaded
- [ ] TTS works (if enabled)
- [ ] Urdu mode works (if enabled)
- [ ] Vercel deployment successful
- [ ] HF Spaces deployment successful
- [ ] Cross-origin requests work in production

---

## Development Timeline

### Phase 1: Core Setup (Week 1)
- [ ] Initialize Docusaurus site
- [ ] Set up FastAPI backend structure
- [ ] Configure Qdrant/Pinecone
- [ ] Implement document ingestion

### Phase 2: RAG Implementation (Week 2)
- [ ] Build embedding pipeline
- [ ] Implement vector search
- [ ] Create chat endpoint
- [ ] Test RAG accuracy

### Phase 3: Frontend Integration (Week 3)
- [ ] Build chat widget component
- [ ] Integrate with backend API
- [ ] Style and UX improvements
- [ ] Add search functionality

### Phase 4: Optional Features (Week 4)
- [ ] TTS integration
- [ ] Urdu mode
- [ ] User authentication
- [ ] ROS2 demos

### Phase 5: Deployment (Week 5)
- [ ] Deploy frontend to Vercel
- [ ] Deploy backend to HF Spaces
- [ ] Configure environment variables
- [ ] End-to-end testing
- [ ] Documentation

---

## Resources

### Documentation
- [Docusaurus](https://docusaurus.io)
- [FastAPI](https://fastapi.tiangolo.com)
- [Qdrant](https://qdrant.tech/documentation)
- [Pinecone](https://docs.pinecone.io)
- [LangChain](https://python.langchain.com)
- [Vercel](https://vercel.com/docs)
- [Hugging Face Spaces](https://huggingface.co/docs/hub/spaces)

### AI Providers
- [OpenAI API](https://platform.openai.com/docs)
- [Anthropic Claude API](https://docs.anthropic.com)
- [ElevenLabs TTS](https://elevenlabs.io/docs)

### Examples
- [Docusaurus Chat Widget Example](https://github.com/facebook/docusaurus/tree/main/examples)
- [FastAPI RAG Example](https://github.com/langchain-ai/langchain/tree/master/templates)

---

## Support & Maintenance

### Monitoring
- Vercel Analytics for frontend
- HF Spaces logs for backend
- Vector DB usage metrics

### Scaling Considerations
- Qdrant: Self-hosted → Cloud for production
- Backend: HF Spaces → Dedicated server if needed
- Caching: Add Redis for frequent queries

### Cost Estimates
- **Vercel**: Free for hobby projects
- **HF Spaces**: Free (with limitations) or $0.60/hr for upgraded
- **Qdrant Cloud**: ~$25/month for 1GB
- **OpenAI API**: ~$0.0001 per 1K tokens (embedding), ~$0.01 per 1K tokens (GPT-4)

---

## Next Steps

1. **Review and refine this spec** using AI agents (Claude/OpenAI)
2. **Generate sub-specs** for each component
3. **Begin implementation** starting with backend ingestion
4. **Iterate and test** each component
5. **Deploy and monitor**

