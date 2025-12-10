import express from 'express';
import cors from 'cors';
import cookieParser from 'cookie-parser';
import {
  signUp,
  signIn,
  signOut,
  getCurrentUserProfile,
  updateUserProfile,
  deleteUserProfile
} from './backend/api.ts';
import { personalizeChapter } from './backend/chapters/api.ts';
import { translateChapter } from './backend/translation/api.ts';
import { requireAuth } from './backend/middleware.ts';

const app = express();
const PORT = process.env.PORT || 4001;  // Changed from 4000 to 4001 to avoid conflicts

// Middleware
app.use(cors({
  origin: process.env.CLIENT_URL || ['http://localhost:3000', 'http://localhost:3001', 'http://localhost:3002', 'http://localhost:3003'],
  credentials: true,
}));
app.use(express.json());
app.use(cookieParser());

// Authentication routes
app.post('/api/auth/signup', signUp);
app.post('/api/auth/signin', signIn);
app.post('/api/auth/signout', signOut);

// Protected routes
app.get('/api/auth/profile', requireAuth, getCurrentUserProfile);
app.put('/api/auth/profile', requireAuth, updateUserProfile);
app.delete('/api/auth/profile', requireAuth, deleteUserProfile);

// Chapter personalization (protected)
app.post('/api/chapters/:id/personalize', requireAuth, personalizeChapter);

// Chapter translation (can be public or protected)
// For content translation, sometimes you might want to allow public access
// but track usage or require auth for caching purposes
app.post('/api/translate', translateChapter); // Public endpoint to allow translation
// If you want to require auth, use: app.post('/api/translate', requireAuth, translateChapter);

// Health check
app.get('/', (req, res) => {
  res.json({ message: 'Better-Auth API server is running' });
});

app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
});