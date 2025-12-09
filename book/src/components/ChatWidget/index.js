import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { HiChatBubbleLeftRight, HiXMark, HiOutlineCursorArrowRays, HiOutlineTrash } from 'react-icons/hi2';
import styles from './styles.module.css';

export default function ChatWidget() {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = siteConfig.themeConfig.apiBaseUrl;
  const API_KEY = siteConfig.themeConfig.apiKey;

  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState('full'); // 'full' or 'selection'
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [selectionLocked, setSelectionLocked] = useState(false); // Lock selection when in selection mode
  const [apiError, setApiError] = useState(false); // New state for API error
  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);
  const inputRef = useRef(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Capture text selection - but don't clear it when clicking on input
  useEffect(() => {
    const handleSelection = (e) => {
      // Don't capture selection if clicking inside the chat widget
      if (chatContainerRef.current && chatContainerRef.current.contains(e.target)) {
        return;
      }
      
      // Don't update if selection is locked (user is in selection mode)
      if (selectionLocked) {
        return;
      }

      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      // Only update if we have a meaningful selection (not empty, not just whitespace)
      if (text && text.length > 10) {
        setSelectedText(text);
      } else if (!selectionLocked) {
        // Only clear if not locked and selection is truly empty
        // Check if there's actually no selection (not just clicking)
        if (selection.rangeCount === 0 || selection.toString().trim() === '') {
          // Don't clear if we're in selection mode - keep the locked selection
          if (mode !== 'selection') {
            setSelectedText('');
          }
        }
      }
    };

    // Use a slight delay to capture selection after mouse/key release
    const handleSelectionDelayed = (e) => {
      setTimeout(() => handleSelection(e), 10);
    };

    document.addEventListener('mouseup', handleSelectionDelayed);
    document.addEventListener('keyup', handleSelectionDelayed);

    return () => {
      document.removeEventListener('mouseup', handleSelectionDelayed);
      document.removeEventListener('keyup', handleSelectionDelayed);
    };
  }, [selectionLocked, mode]);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);
    setApiError(false);

    try {
      let response;
      
      // Build headers with optional API key
      const headers = {
        'Content-Type': 'application/json',
      };
      
      if (API_KEY) {
        headers['X-API-Key'] = API_KEY;
      }
      
      if (mode === 'selection' && selectedText) {
        // Selection mode - use highlight_query endpoint
        response = await fetch(`${API_BASE_URL}/highlight_query`, {
          method: 'POST',
          headers: headers,
          body: JSON.stringify({
            question: userMessage,
            selected_text: selectedText,
          }),
        });
      } else {
        // Full corpus mode - use query endpoint
        response = await fetch(`${API_BASE_URL}/query`, {
          method: 'POST',
          headers: headers,
          body: JSON.stringify({
            question: userMessage,
            top_k: 5, // Optional: number of chunks to retrieve
          }),
        });
      }

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`API error: ${response.status} - ${errorText}`);
      }

      const data = await response.json();
      
      // Handle response based on mode
      if (mode === 'selection') {
        // Highlight query returns: { answer, source_context }
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: data.answer,
            source_context: data.source_context,
          },
        ]);
      } else {
        // Query returns: { answer, sources, chunks_used }
        // Convert sources array to citation format
        const citations = (data.sources || []).map((source, idx) => ({
          url: `/docs/${source.replace(/\.md$/, '')}`,
          file_path: source,
          section_id: source.split('/').pop().replace(/\.md$/, ''),
        }));
        
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: data.answer,
            citations: citations,
            chunks_used: data.chunks_used || 0,
          },
        ]);
      }
    } catch (error) {
      console.error('Chat error:', error);
      // Restore user input and show specific error message
      setInput(userMessage); 
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'The chatbot is currently unavailable. Please try again later.',
          error: true,
        },
      ]);
      setApiError(true);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleSelectionMode = () => {
    // Capture current selection if available
    const selection = window.getSelection();
    const text = selection.toString().trim();
    
    if (text && text.length > 10) {
      setSelectedText(text);
      setSelectionLocked(true); // Lock the selection
      setMode('selection');
      setIsOpen(true);
    } else if (selectedText) {
      // Use previously captured selection
      setSelectionLocked(true);
      setMode('selection');
      setIsOpen(true);
    } else {
      alert('Please select some text on the page first, then click this button.');
      return;
    }
  };

  const handleModeChange = (newMode) => {
    if (newMode === 'selection') {
      // Capture current selection when switching to selection mode
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text && text.length > 10) {
        setSelectedText(text);
        setSelectionLocked(true);
      } else if (!selectedText) {
        alert('Please select text on the page first');
        return;
      } else {
        setSelectionLocked(true); // Lock existing selection
      }
    } else {
      setSelectionLocked(false); // Unlock when switching to full mode
    }
    setMode(newMode);
  };

  const clearSelection = () => {
    setSelectedText('');
    setSelectionLocked(false);
    if (mode === 'selection') {
      setMode('full');
    }
  };

  return (
    <>
      {/* Floating action buttons */}
      <div className={styles.floatingButtons}>
        {selectedText && (
          <button
            className={styles.selectionButton}
            onClick={handleSelectionMode}
            title="Ask about selected text"
          >
            <HiOutlineCursorArrowRays className={styles.buttonIcon} />
            <span>Ask from selection</span>
          </button>
        )}
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(!isOpen)}
          title={isOpen ? "Close chat" : "Open chat"}
        >
          {isOpen ? (
            <>
              <HiXMark className={styles.buttonIcon} />
              <span>Close</span>
            </>
          ) : (
            <>
              <HiChatBubbleLeftRight className={styles.buttonIcon} />
              <span>Chat</span>
            </>
          )}
        </button>
      </div>

      {/* Chat panel */}
      {isOpen && (
        <div className={styles.chatPanel} ref={chatContainerRef}>
          <div className={styles.chatHeader}>
            <h3>Textbook Chatbot</h3>
            <div className={styles.modeToggle}>
              <button
                className={mode === 'full' ? styles.active : ''}
                onClick={() => handleModeChange('full')}
              >
                Ask the book
              </button>
              <button
                className={mode === 'selection' ? styles.active : ''}
                onClick={() => handleModeChange('selection')}
              >
                Ask selected text
              </button>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              <HiXMark />
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>
                  {mode === 'full'
                    ? 'Ask me anything about the Physical AI & Humanoid Robotics textbook!'
                    : 'Ask questions about the selected text. I will only use that text to answer.'}
                </p>
                {mode === 'selection' && !selectedText && (
                  <p className={styles.warning}>
                    No text selected. Please select text on the page first.
                  </p>
                )}
                {mode === 'selection' && selectedText && (
                  <div className={styles.selectedTextPreview}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                      <strong>Selected text:</strong>
                      <button
                        onClick={clearSelection}
                        className={styles.clearSelectionButton}
                        title="Clear selection"
                      >
                        <HiOutlineTrash className={styles.clearIcon} />
                        <span>Clear</span>
                      </button>
                    </div>
                    <p>{selectedText.substring(0, 150)}...</p>
                  </div>
                )}
              </div>
            )}
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={
                  msg.role === 'user'
                    ? styles.userMessage
                    : styles.assistantMessage
                }
              >
                <div className={styles.messageContent}>
                  {msg.content}
                  {msg.citations && msg.citations.length > 0 && (
                    <div className={styles.citations}>
                      <strong>Sources:</strong>
                      {msg.citations.map((citation, cIdx) => (
                        <a
                          key={cIdx}
                          href={citation.url}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.citationLink}
                        >
                          {citation.section_id || citation.file_path || 'View source'}
                        </a>
                      ))}
                    </div>
                  )}
                  {msg.source_context && mode === 'selection' && (
                    <div className={styles.citations}>
                      <strong>Context used:</strong>
                      <div style={{ 
                        marginTop: '8px', 
                        padding: '8px', 
                        background: 'rgba(0,0,0,0.05)',
                        borderRadius: '4px',
                        fontSize: '12px',
                        fontStyle: 'italic'
                      }}>
                        {msg.source_context.substring(0, 200)}
                        {msg.source_context.length > 200 ? '...' : ''}
                      </div>
                    </div>
                  )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={styles.assistantMessage}>
                <div className={styles.loading}>Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              onFocus={() => {
                // Don't clear selection when focusing on input
                // Selection is already locked if in selection mode
              }}
              placeholder={
                mode === 'selection' && !selectedText
                  ? 'Select text first...'
                  : 'Type your question...'
              }
              disabled={isLoading || (mode === 'selection' && !selectedText)}
              rows={2}
              className={styles.input}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim() || (mode === 'selection' && !selectedText)}
              className={styles.sendButton}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
}

