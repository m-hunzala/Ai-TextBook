const express = require('express');
const path = require('path');
const httpProxyMiddleware = require('http-proxy-middleware');

module.exports = (req, res, next) => {
  // Create a proxy for auth API requests to the auth server running on port 4000
  if (req.url.startsWith('/api/auth')) {
    const proxy = httpProxyMiddleware.createProxyMiddleware({
      target: process.env.AUTH_API_URL || 'http://localhost:4000',
      changeOrigin: true,
      logLevel: 'debug',
      onProxyReq: (proxyReq, req, res) => {
        console.log(`Proxying request: ${req.method} ${req.url} -> ${process.env.AUTH_API_URL || 'http://localhost:4000'}${req.url}`);
      },
      onProxyRes: (proxyRes, req, res) => {
        console.log(`Proxying response: ${req.url} -> Status: ${proxyRes.statusCode}`);
      }
    });
    return proxy(req, res, next);
  }
  
  // For other requests, let Docusaurus handle them
  next();
};