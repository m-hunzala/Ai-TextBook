// api/auth.js - Docusaurus API routes for Better-Auth integration

import express from 'express';
import { auth } from '../../auth/backend/auth.ts';

const router = express.Router();

// Forward Better-Auth API routes to the auth server running on port 4000
router.all('/auth/*', async (req, res) => {
  try {
    // Get the auth server URL from environment variables
    const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4000';
    
    // Forward the request to the auth server
    const response = await fetch(`${authServerUrl}${req.url}`, {
      method: req.method,
      headers: {
        'Content-Type': 'application/json',
        ...req.headers,
      },
      body: req.body ? JSON.stringify(req.body) : undefined,
    });
    
    // Forward the response from auth server
    const data = await response.json();
    res.status(response.status).json(data);
  } catch (error) {
    console.error('Auth API error:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

export default router;