#ifndef OBJ_LEXER_H
#define OBJ_LEXER_H

#include <string>
#include <fstream>

#define MAX_STRING 50

/**
 * @brief Helper class convering Wavefront obj format into
 */
class OBJLexer {
public:
	/**
	 * OBJ file format tokens identificators.
	 */
	enum Token {
		TOK_EOL,
		TOK_STRING,
		TOK_SLASH,
		TOK_NUMBER,
	};

	/**
	 * @brief Constructor
	 *
	 * @param[in] path Wavefront format file path.
	 */
	OBJLexer(const char *path);
	OBJLexer(const std::string &path);

	virtual ~OBJLexer ();

	/**
	 * @brief Splits file content into tokens.
	 *
	 * Function stops after encountering and parsing each token and advances file offset.
	 * Should be called repeatedly unless error occured or TOK_EOF has been
	 * returned.
	 * Depending on token type different class members may have been updated.
	 *
	 * TOK_STRING => GetString returns newly parsed string.
	 * TOK_SLASH => no data
	 * TOK_NUMBER => GetValue returns newly parsed value.
	 * TOK_EOL => GetLine returns new line number. (1 - based indexing)
	 *
	 * @return True if next token was successfully parsed. False if EOL was
	 * reached or error has occured. (Check error status with GetError)
	 */
	bool ProcessNext(void);

	/**
	 * @brief Returns a character content of parsed token
	 * string is only valid if current token is TOK_STRING
	 */
	const std::string &GetString(void) { return m_str; }

	/**
	 * @brief Returns current line in parserd file.
	 * value is only valid if current token is TOK_EOL
	 */
	unsigned int GetLine(void) { return m_line; }

	/**
	 * @brief Returns a float value of parsed token
	 */
	float GetValue(void) { return m_value; }

	/**
	 * @brief Gets error message. Only valid if current
	 * token is TOK_ERROR.
	 *
	 * @return NULL if no error has occured or error message.
	 */
	const char *GetError(void) { return m_error ? m_str.c_str() : NULL; }

	/**
	 * @brief Gets last parsed token.
	 */
	Token GetToken(void) { return m_token; }

private:
	void SkipWS(void);
	Token m_token;
	int m_line;
	float m_value;
	bool m_error;
	std::string m_str;
	std::fstream m_file;
};

#endif
