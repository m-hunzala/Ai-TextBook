import React, { useState } from "react";
import { GoogleGenerativeAI } from "@google/generative-ai";

export default function Chatbot() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");

  // Initialize the Gemini AI with the API key
  const genAI = new GoogleGenerativeAI("AIzaSyC3G3JW_gXaPx2uuB07THJq8kPs_RiM_jw");
  const model = genAI.getGenerativeModel({ model: "gemini-1.5-flash" });

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: "user", text: input };
    setMessages((prev) => [...prev, userMessage]);
    setInput("");

    try {
      const result = await model.generateContent(input);
      const botReply = result.response.text();

      setMessages((prev) => [
        ...prev,
        { role: "bot", text: botReply },
      ]);
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        { role: "bot", text: "❌ Error: Unable to get response." },
      ]);
      console.error(error);
    }
  };

  return (
    <>
      <div
        onClick={() => setOpen(true)}
        style={{
          position: "fixed",
          bottom: "20px",
          right: "20px",
          background: "#2563eb",
          color: "white",
          padding: "15px",
          borderRadius: "50%",
          cursor: "pointer",
          boxShadow: "0 4px 12px rgba(0,0,0,0.2)",
          fontSize: "20px",
        }}
      >
        💬
      </div>

      {open && (
        <div
          style={{
            position: "fixed",
            bottom: "90px",
            right: "20px",
            width: "350px",
            height: "450px",
            background: "white",
            borderRadius: "15px",
            boxShadow: "0 6px 20px rgba(0,0,0,0.25)",
            padding: "15px",
            display: "flex",
            flexDirection: "column",
          }}
        >
          <div
            style={{
              fontWeight: "bold",
              marginBottom: "10px",
              display: "flex",
              justifyContent: "space-between",
            }}
          >
            AI Chatbot
            <span
              style={{ cursor: "pointer", color: "red" }}
              onClick={() => setOpen(false)}
            >
              ✖
            </span>
          </div>

          <div
            style={{
              flex: 1,
              overflowY: "auto",
              marginBottom: "10px",
              padding: "5px",
              background: "#f1f5f9",
              borderRadius: "8px",
            }}
          >
            {messages.map((msg, i) => (
              <div
                key={i}
                style={{
                  marginBottom: "8px",
                  textAlign: msg.role === "user" ? "right" : "left",
                }}
              >
                <span
                  style={{
                    display: "inline-block",
                    padding: "8px 12px",
                    borderRadius: "10px",
                    background:
                      msg.role === "user" ? "#2563eb" : "#e2e8f0",
                    color: msg.role === "user" ? "white" : "black",
                  }}
                >
                  {msg.text}
                </span>
              </div>
            ))}
          </div>

          <div style={{ display: "flex", gap: "5px" }}>
            <input
              style={{
                flex: 1,
                padding: "10px",
                borderRadius: "8px",
                border: "1px solid #cbd5e1",
              }}
              placeholder="Type something..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && sendMessage()}
            />
            <button
              onClick={sendMessage}
              style={{
                padding: "10px 15px",
                background: "#2563eb",
                color: "white",
                borderRadius: "8px",
                border: "none",
              }}
            >
              ➤
            </button>
          </div>
        </div>
      )}
    </>
  );
}