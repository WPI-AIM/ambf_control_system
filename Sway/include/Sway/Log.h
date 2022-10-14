#pragma once
#include "PCH/pch.h"

namespace Sway {

	class Log
	{
	public:
		static void Init();

		inline static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
		inline static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }
	private:
		static std::shared_ptr<spdlog::logger> s_CoreLogger;
		static std::shared_ptr<spdlog::logger> s_ClientLogger;
	};
}

// Core Log macros
#define SW_CORE_TRACE(...)	::Sway::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define SW_CORE_DEBUG(...)  ::Sway::Log::GetCoreLogger()->debug(__VA_ARGS__)
#define SW_CORE_INFO(...)		::Sway::Log::GetCoreLogger()->info(__VA_ARGS__)
#define SW_CORE_WARN(...)		::Sway::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define SW_CORE_ERROR(...)	::Sway::Log::GetCoreLogger()->error(__VA_ARGS__)

// Client Log macros
#define SW_CLIENT_TRACE(...)::Sway::Log::GetClientLogger()->trace(__VA_ARGS__)
#define SW_CLIENT_DEBUG(...)::Sway::Log::GetClientLogger()->debug(__VA_ARGS__)
#define SW_CLIENT_INFO(...)	::Sway::Log::GetClientLogger()->info(__VA_ARGS__)
#define SW_CLIENT_WARN(...)	::Sway::Log::GetClientLogger()->warn(__VA_ARGS__)
#define SW_CLIENT_ERROR(...)::Sway::Log::GetClientLogger()->error(__VA_ARGS__)
