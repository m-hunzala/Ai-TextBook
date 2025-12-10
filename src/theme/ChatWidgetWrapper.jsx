import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import config from '../config';

// Wrapper component for easy Docusaurus integration
// This can be used in Docusaurus theme components
const ChatWidgetWrapper = () => {
  // Get API URL and key from configuration
  const apiUrl = config.API_URL;
  const apiKey = config.API_KEY;

  // Show the chat widget even if API key is not configured to ensure it appears on the site
  // The chat widget itself will handle the missing API key case
  return (
    <ChatWidget
      apiUrl={apiUrl}
      apiKey={apiKey}
    />
  );
};

export default ChatWidgetWrapper;