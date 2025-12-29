<!--
    Sync Impact Report:
    - Version change: N/A -> 1.0.0 (new Phase 2 constitution)
    - List of modified principles: N/A (new document)
    - Added sections: All sections for Phase 2 RAG chatbot implementation
    - Removed sections: N/A
    - Templates requiring updates:
        - ⚠ .specify/templates/plan-template.md (Manual check recommended for Phase 2 alignment)
        - ⚠ .specify/templates/spec-template.md (Manual check recommended for Phase 2 alignment)
        - ⚠ .specify/templates/tasks-template.md (Manual check recommended for Phase 2 alignment)
        - ⚠ README.md (Manual check recommended for Phase 2 alignment)
    - Follow-up TODOs:
        - TODO(RATIFICATION_DATE): Determine original adoption date for Phase 2.
-->

# Constitution for Spec-Driven Development of the RAG-Integrated Chatbot for the "Physical AI & Humanoid Robotics" Textbook

## Vision
The vision of this Phase 2 project is to create a separate Phase 2 constitution (e.g., phase2-constitution.md) that builds on/references Phase 1 without overwriting—avoids conflicts, preserves immutability, and allows frontend (Docusaurus book) to read Phase 1 cleanly. Overwrite risks breaking references; separate is better for spec-driven flow. Within this framework, forge a seamless, AI-native Retrieval-Augmented Generation (RAG) chatbot ecosystem that transforms the spec-driven Docusaurus textbook from Phase 1 into a dynamic, interactive intelligence platform. This chatbot will empower users to query the "Teaching Physical AI & Humanoid Robotics Course" content with precision, drawing exclusively from the book's rich, stratified knowledge base to deliver grounded, cited responses. By integrating reusable Claude Code subagents and agent skills, the system will embody a mixture of experts (MoE) architecture, where specialized agents collaborate for efficient ingestion, retrieval, and generation—mirroring the embodied intelligence theme of the course itself. The chatbot will support advanced user interactions, such as text selection-triggered pop-up searches for contextual queries (e.g., selecting a sentence/word/paragraph in the Docusaurus frontend triggers a search bar; upon submission, the bot entertains precisely from the book), personalization based on user background (gathered via Better-Auth sign-up questions like Python proficiency, hardware access—e.g., NVIDIA GPU ownership—to adapt content depth), one-click Urdu translation (saved as user preference post-sign-in), auto-generated summaries/quizzes/learning boosters per chapter, and secure, modular backend deployment. Ultimately, this creates a REAL AI-powered education platform that feels alive, responsive, and tailored—not just a query tool—democratizing access to Physical AI expertise while adhering to ethical AI practices, cybersecurity, and critical-sector safety (e.g., no assistance in disallowed activities like hacking or weapons).

## Core Principles

### RAG-Centric Intelligence with MoE Integration
All responses must be grounded in the Phase 1 textbook content (Markdown files from Docusaurus), using Qdrant for semantic vector search (Cohere embeddings, with Hugging Face alternatives like Sentence Transformers for local/offline/fine-tuned models) and Neon for structured data (e.g., user profiles, metadata). Employ a mixture of experts via Claude Code subagents (.claude/agents for expert instructions, e.g., Indexer Agent for ingestion, Retriever Agent for hybrid search, Answerer Agent for multi-hop reasoning, Manager Agent for collection health) and reusable skills (.claude/skills for task-specific prompts, e.g., smart chunking with 500-token max/50 overlap, preserving code blocks, hybrid vector/keyword search with re-ranking). Agents collaborate iteratively (e.g., Retriever fetches chunks, Answerer constructs cited prompts, handling "I don't know" for insufficient context).

### User-Centric Interactivity and Personalization
Frontend (ChatKit-JS integrated into Docusaurus) enables seamless querying: General questions via chat interface; selected text (sentence/word/paragraph) triggers pop-up search bar—upon search button click, passes selection as query context to backend for precise, book-grounded answers. Post-sign-in via Better-Auth, ask background questions (e.g., "Do you know Python? Level: Beginner/Intermediate/Advanced"; "Hardware: NVIDIA GPU? Jetson? Cloud access?") to personalize content (e.g., skip basics for pros, emphasize GPU optimizations for advanced users). One-click Urdu translation button per chapter (saved preference, applied via backend processing). Auto-generate summaries/quizzes/boosters using OpenAI-Agents-SDK (free models like o1-mini) for engagement.

### Modular, Secure Backend Architecture
Use FastAPI for backend API (routes for ingestion/retrieval/Q&A/auth), OpenAI-Agents-SDK for agent orchestration (free tier models), Cohere/Hugging Face for embeddings (e.g., Hugging Face's all-MiniLM-L6-v2 for lightweight local processing), Qdrant Cloud Free Tier for vector DB (cosine similarity, payload indexing for filters like chapter/section/tags), Neon Serverless Postgres for structured data (user profiles, preferences, metadata). Deploy to Railway for scalability. Folder structure: /backend (FastAPI main), /rag (indexing/retrieval logic with Hugging Face integration), /agents (Claude subagent configs), /skills (reusable prompts), /website (Docusaurus integration points). Ensure cybersecurity: Input sanitization to prevent injections/SQL attacks, encryption for Neon data (user backgrounds), adversarial robustness in RAG (e.g., confidence scores, source citations), zero-trust for API endpoints. Handle critical scenarios: Low-end devices (client-side fallbacks), errors (graceful failures with messages, health checks/logging), token limits (phased processing, batch ops).

### Reusable Intelligence via Claude Code
Define skills in .claude/skills/{name}.md with clear descriptions, examples, dependencies (.env vars like API keys, Hugging Face tokens), error handling (graceful failures, timestamps), and integration points (e.g., Docusaurus Markdown parsing with frontmatter for metadata/tags, multi-language support). Skills: Rag-Indexer (scan docs, smart chunking/overlap, Cohere/Hugging Face embeddings, incremental updates, batch progress; tools: Read/Glob/Bash/WebFetch); Rag-Retriever (semantic/hybrid search, re-rank, filters/metadata, top_k/threshold; tools: Bash/Read); Rag-Answerer (fetch chunks, construct prompts, cited answers, multi-hop/"I don't know", confidence/sources; tools: Bash/Read); Rag-Manager (create/delete collections, update settings (vector size/cosine), stats/export/import, health/diagnostics, clear stale; tools: Bash). Support multiple embeddings (OpenAI/Cohere/Local Sentence Transformers via Hugging Face), chunking (semantic via headers, preserve code), Qdrant schema (metadata fields, payload indexing). Agents in .claude/agents: Expert instructions for each (e.g., Indexer Agent: "You are an expert in document ingestion... use Rag-Indexer skill with Hugging Face for local embeddings...").

### Hands-On, Multi-Level Focus with Ethical Safeguards
Content handling mirrors course themes (e.g., VLA queries use Whisper-like prompts). Strata for all levels: Beginners (simple explanations), Basics/Normal (step-by-step), Pro/Advanced (optimizations/cutting-edge like 2025 Isaac updates), Researchers (analyses/open problems). Prioritize efficiency: Accurate/cited/grounded answers, short responses for refusals. Align with disallowed activities ban (e.g., no hacking/social engineering assistance via RAG filters).

### Deployment and Scalability Principles
Use Railway for backend hosting (free tier compatible, auto-scaling). Frontend embeds ChatKit-JS for real-time UI (pop-up search on text select, button for personalization/translation). All data stored cleanly in Neon (structured user data) + Qdrant (vectors). Reusable via /agents (expert MoE) and /skills (task modular). Support low-end devices (minimal JS, async ops).

## Success Criteria

Functional Completeness: Chatbot answers general/book-selected queries accurately from Phase 1 content (95% grounded, cited); pop-up search on text select passes context efficiently; personalization adapts content (e.g., chapter buttons show tailored strata); Urdu translation high-quality/instant/saved.

Performance and UX: Clean/minimal UI (no extra animations), fast loading (<2s), mobile-friendly; RAG answers precise/efficient (<10s latency), with confidence/sources; quizzes/summaries auto-generated per chapter.

Security and Reliability: Better-Auth sign-up/sign-in works (background questions stored securely in Neon); no vulnerabilities (e.g., CSP headers, encrypted DB); error handling (health checks, logging); works on free tiers (Qdrant/Neon/Railway).

Reusability and MoE Efficacy: Skills/agents in .claude reusable (batch/interactive modes, progress feedback); MoE handles complex queries (e.g., multi-hop via iterative retrieval).

Deployment Stability: Backend on Railway, integrated with Docusaurus; live URLs functional; bonus 50 points for Better-Auth personalization.

User Metrics: Readable <45min total (book+chat); accurate RAG (chunking/MiniLM embeddings mitigate low accuracy); minimal confusion (clean UI).

## Constraints

Resource Limits: Free tiers only (Qdrant Cloud, Neon Serverless, OpenAI-Agents free models, Cohere free embeddings, Railway hobby tier); avoid heavy dependencies (minimal npm/Python libs); support low-end devices (phones, no RTX).

Scope Non-Goals: No extra animations/motion; no long/complex code (educational focus); no unrelated features (e.g., no external searches—book-only RAG); no complex robotics sims in chat.

Technical: Backend modular (FastAPI routes/services); data in Neon/Qdrant cleanly; async/batch for performance; error grace (clear messages); observability (timestamps/logs).

Ethical: Assume good intent; no moralizing; resist jailbreaks; enforce disallowed activities refusal (short responses); cybersecurity in AI (adversarial defenses, secure auth).

## Stakeholders

Learners (Primary): Beginners to researchers querying book; needs: Precise answers, personalization (e.g., hardware-based), Urdu support.

Educators: Use for teaching/assessments; needs: Quizzes/summaries, secure student data.

Hackathon Judges: Evaluate RAG accuracy, auth/personalization (bonus points), deployment.

Developers (Us): Build with Claude/Spec-Kit; needs: Reusable skills/agents for efficiency.

Community: Contribute/open-source; needs: Modular/extensible (MoE agents/skills).

Tech Providers: OpenAI (agents/ChatKit), Cohere (embeddings), Qdrant/Neon (DBs), Better-Auth (auth), FastAPI/Railway (backend).

## Brand Voice and Style

Tone: Enthusiastic, empowering mentor—guiding users like a robotics expert (e.g., "Let's dive into how ROS 2 bridges your AI code to humanoid control!").

Style: Clear/concise/visual (Markdown headers, code blocks, tables for comparisons); active voice; inclusive (gender-neutral, culturally sensitive with Urdu).

Consistency: Docusaurus conventions; brand as "Panaversity Physical AI Chatbot" with modern aesthetics (minimalist, fast).

ML Analyst Infusion: Weave insights (e.g., "Think of RAG as fine-tuning your query with book embeddings for precise embodied AI answers").

## Mission (Reiterated)
Build a fully AI-native RAG chatbot integrated with the Phase 1 textbook, using MoE via Claude agents/skills, ChatKit/FastAPI backend, Qdrant/Neon DBs, Better-Auth, and Railway deployment. Fast, simple, beautiful—feels like REAL AI education, with text-select pop-ups, personalization, Urdu, summaries/quizzes.

## Architecture Principles (Detailed for Implementation)

Folder Structure: /backend (FastAPI app.py/routes), /rag (indexing/retrieval logic), /agents (Claude subagent configs), /skills (reusable prompts), /website (Docusaurus integration points).

Content Flow: Index Docusaurus Markdown (Rag-Indexer scans /docs, chunks semantically, embeds Cohere, uploads Qdrant with metadata—file/path/section/title/tags; incremental for updates). Retrieve (hybrid search, filters, re-rank). Answer (context prompts, citations, multi-hop). Manage (collections/schema, health).

Frontend Integration: ChatKit-JS for UI (general chat, text-select pop-up: on select, show bar; submit to backend /rag endpoint). Personalization: Post-auth, Neon stores background; adapt responses (e.g., pro users get advanced code). Translation: Button triggers backend re-render in Urdu (save pref).

MoE Execution: Agents chain skills (e.g., Query → Retriever Agent uses Rag-Retriever → Answerer uses Rag-Answerer; Manager for diagnostics).

Deployment: Railway (env vars for keys/DB URIs); free tiers; health checks/logging for errors.

## Risks & Mitigation

RAG Low Accuracy: Mitigate with smart chunking/MiniLM/Cohere, hybrid search, re-ranking; test with multi-hop.

Token/Usage High: Phased/batch processing; free models.

User Confusion: Minimal UI, clear pop-ups; short refusals.

Backend Errors: Health checks/logging; graceful failures.

Security Breaches: Sanitize inputs; encrypt Neon; adversarial filters.

Connectivity/DB Issues: Free tier fallbacks; async ops.

## Definition of Done

All skills/agents defined (.claude folders).
Backend deployed (Railway, FastAPI/Qdrant/Neon/ChatKit-Python).
Frontend integrated (Docusaurus with ChatKit-JS pop-up/search/personalization/translation).
RAG functional: Grounded answers, cited; personalization visible; Urdu fast.
Auth working (Better-Auth sign-up questions).
Quizzes/summaries per chapter.
Deployed URLs live/stable; MoE handles queries efficiently.

## Governance
This constitution serves as the guiding prompt for Spec-Kit Plus to complete Phase 2: Generate the RAG chatbot ecosystem, ensuring all specified elements are richly detailed and aligned with the vision. Now, proceed to specify.md for detailed specs on agents/skills/backend.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Determine original adoption date for Phase 2. | **Last Amended**: 2025-12-18