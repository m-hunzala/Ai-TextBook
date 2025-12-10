const express = require('express');
const cors = require('cors');
const cookieParser = require('cookie-parser');
const translationRoutes = require('./api/translationRoutes');

const app = express();
const PORT = process.env.PORT || 8003;

// Middleware
app.use(cors({
  origin: process.env.RAG_BACKEND_URL || 'http://localhost:3000', // Docusaurus default
  credentials: true
}));
app.use(express.json({ limit: '10mb' }));
app.use(cookieParser());

// Routes
app.use('/api/translate', translationRoutes);

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'OK', service: 'Translator Agent API' });
});

// Error handling middleware
app.use((err, req, res, next) => {
  console.error('Server error:', err);
  res.status(500).json({ error: 'Internal server error' });
});

// 404 handler
app.use('*', (req, res) => {
  res.status(404).json({ error: 'Route not found' });
});

app.listen(PORT, () => {
  console.log(`Translator Agent API server running on port ${PORT}`);
});

module.exports = app;