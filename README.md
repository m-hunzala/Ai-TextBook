# Docusaurus Book with AI Agents

This project implements a Docusaurus book app with AI agents, authentication, personalization, and Urdu translation capabilities.

## Features

- **AI Agents**: Summarizer, Code Runner, and Translator microservices
- **Authentication**: Better-Auth with custom signup fields
- **Personalization**: Chapter content tailored to user profile
- **Translation**: Urdu translation preserving markdown structure
- **Docusaurus Integration**: Seamless integration with Docusaurus documentation site

## Prerequisites

- Node.js 24+
- Python 3.8+
- PostgreSQL database
- npm

## Setup Instructions

### 1. Install Dependencies

```bash
# Install main dependencies
npm install

# Install agent dependencies
cd agents/summarizer && pip install -r requirements.txt
cd ../code_runner && pip install -r requirements.txt
cd ../translator && pip install -r requirements.txt

# Install Docusaurus site dependencies (if not already installed)
cd my-book && npm install && cd ..
```

### 2. Environment Configuration

Create a `.env` file in the `auth` directory:

```bash
# auth/.env
DATABASE_URL=postgresql://user:password@localhost:5432/docusaurus_auth
SECRET_KEY=your-super-secret-key
AGENT_TRANSLATOR_URL=http://localhost:8003
TRANSLATION_PROVIDER=agent
```

### 3. Database Setup

Create the required database tables:

```sql
-- PostgreSQL setup
CREATE TABLE user_metadata (
  id SERIAL PRIMARY KEY,
  user_id TEXT UNIQUE NOT NULL,
  software_background TEXT,
  hardware_available JSON,
  experience_level TEXT,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  updated_at TIMESTAMP DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_user_metadata_user_id ON user_metadata(user_id);
```

### 4. Running the System

First, install dependencies:

```bash
npm install
```

To start the AI agents and authentication server:

```bash
npm start
```

This will start:
- Summarizer Agent (localhost:8001)
- Code Runner Agent (localhost:8002)
- Translator Agent (localhost:8003)
- Authentication Server (localhost:4000)

To start Docusaurus separately (in another terminal):

```bash
cd my-book
npm start
```

This will start Docusaurus on localhost:3000

Alternatively, you can start services individually:

```bash
# Start agents only
npm run start:agents

# Start auth server only
npm run start:auth

# Start Docusaurus only
cd my-book && npm start
```

## API Endpoints

### Authentication
- `POST /api/auth/signup` - User registration
- `POST /api/auth/signin` - User login
- `POST /api/auth/signout` - User logout
- `GET /api/auth/profile` - Get user profile

### Chapter Personalization
- `POST /api/chapters/:id/personalize` - Personalize chapter content

### Translation
- `POST /api/translate` - Translate chapter to Urdu

### AI Agents
- Summarizer: `POST http://localhost:8001/summarize`
- Code Runner: `POST http://localhost:8002/run_code`
- Translator: `POST http://localhost:8003/translate`

## Development

For development mode with auto-restart:

```bash
npm run dev
```

## Architecture

- **Frontend**: Docusaurus with React components for auth and personalization
- **Backend**: Express.js server with Better-Auth integration
- **AI Agents**: FastAPI microservices running on separate ports
- **Database**: PostgreSQL for user profiles and cached translations

## Customization

### Adding New Agents
1. Create a new directory in `/agents`
2. Implement a FastAPI service
3. Add to the `package.json` start scripts

### Extending Personalization
1. Modify the rules in `/auth/backend/chapters/personalization.ts`
2. Add new content markers in your markdown files
3. Update transformation logic as needed

## Troubleshooting

1. **Agent ports in use**: Change ports in service files and server.ts
2. **Database connection issues**: Verify DATABASE_URL in .env
3. **Translation fails**: Ensure TranslatorAgent is running on port 8003

## File Structure

```
├── agents/                 # AI microservices
│   ├── summarizer/         # Summarizer agent
│   ├── code_runner/        # Code execution agent
│   └── translator/         # Translation agent
├── auth/                  # Authentication system
│   ├── backend/           # Server-side auth logic
│   └── frontend/          # Client-side auth components
├── my-book/              # Docusaurus documentation site
└── package.json          # Main project configuration
```

## Next Steps

1. Customize the personalization rules in `/auth/backend/chapters/personalization.ts`
2. Add more sophisticated translation logic if needed
3. Extend the Docusaurus site with your content
4. Deploy to your preferred hosting platform