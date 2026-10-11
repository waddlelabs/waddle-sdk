// Apache-2.0. Native software-only acceptance for the reviewed rsutils patch.
// Compile with C++14 and the selected source's third-party/rsutils/include.
#include <rsutils/deferred.h>
#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <type_traits>

static int assertions = 0;

void check( bool condition, char const * reason )
{
    ++assertions;
    if( ! condition )
    {
        std::cerr << reason << '\n';
        std::exit( 1 );
    }
}

int main()
{
    static_assert( std::is_nothrow_move_constructible<rsutils::deferred>::value, "Retain no-throw moves" );
    static_assert( std::is_nothrow_move_assignable<rsutils::deferred>::value, "Retain no-throw assignment" );
    for( bool large_callback : { false, true } )
    {
        int calls = 0;
        auto callback = [&calls] { ++calls; };
        auto make_callback = [&]() -> rsutils::deferred::fn
        {
            if( large_callback )
            {
                std::array<int, 64> storage{};
                return [storage, callback] { callback(); (void) storage; };
            }
            return callback;
        };
        rsutils::deferred::fn transferred;
        {
            rsutils::deferred source( make_callback() );
            transferred = source.detach();
            check( ! source.is_valid(), "detach retains a second cleanup owner" );
        }
        check( calls == 0, "detached source executes cleanup prematurely" );
        transferred();
        check( calls == 1, "transferred cleanup does not execute exactly once" );
        calls = 0;
        {
            rsutils::deferred source( make_callback() );
            {
                rsutils::deferred target( std::move( source ) );
                check( ! source.is_valid() && target.is_valid(), "move construction retains a second owner" );
            }
            check( calls == 1, "moved target does not execute cleanup" );
        }
        check( calls == 1, "move construction executes cleanup twice" );
        calls = 0;
        {
            rsutils::deferred source( make_callback() );
            rsutils::deferred target;
            target = std::move( source );
            check( ! source.is_valid() && target.is_valid(), "move assignment retains a second owner" );
            rsutils::deferred * same = &target;
            target = std::move( *same );
            check( target.is_valid(), "self move loses its cleanup owner" );
        }
        check( calls == 1, "move assignment does not execute cleanup exactly once" );
        calls = 0;
        {
            rsutils::deferred canceled( make_callback() );
            canceled.reset();
        }
        check( calls == 0, "reset executes canceled cleanup" );
        auto owned = std::make_shared<int>( 42 );
        {
            rsutils::deferred source( [owned] {} );
            auto target = source.detach();
            check( owned.use_count() == 2, "detach retains callback captures in its source" );
            target();
        }
        check( owned.use_count() == 1, "transferred callback captures leak" );
    }
    std::cout << assertions << " cleanup ownership assertions pass\n";
}
